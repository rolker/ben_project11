"""Configure the stock bt_navigator with BEN's installed bt_task_navigator parameters.

The parameters are built the way the launch files build them: the
installed config/nav2_params.yaml goes through nav2_bringup_launch.py's own
ReplaceString (namespace "ben", use_namespace true), then through the
parameter list of the bt_task_navigator LifecycleNode in
navigation_launch.py (for Nav2 older than 1.4, the error_code_names that
legacy_error_code_params adds, then its RewrittenYaml file), passed to
the node in that order. The BEN
navigator plugin and behaviour-tree library are replaced by the stock
navigate_to_pose navigator, since they are not available here; the
error-code handling under test is in Nav2's BtActionServer, which BEN's
navigator uses unchanged.

The node is started as /ben/bt_task_navigator, configured through its
lifecycle service, and its error-code parameters are read back:
- Nav2 1.4 and later: error_code_name_prefixes is [compute_path,
  follow_path, hover] and error_code_names is not set (it would make
  configure fail);
- Nav2 older than 1.4: error_code_names is [compute_path_error_code,
  follow_path_error_code, hover_error_code].

CMake runs this with ament_add_ros_isolated_pytest_test and passes
BEN_PROJECT11_INSTALL_PREFIX, which the test puts first on
AMENT_PREFIX_PATH (nav2_bringup_launch.py looks up this package's share
directory when loaded); nav2_bt_navigator and nav2_bringup are found on
the ament index of the sourced environment.
"""

import os
import pathlib
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources.python_launch_file_utilities import (
    load_python_launch_file_as_module,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node as LaunchNode
from launch_ros.descriptions import ParameterFile
from launch_ros.utilities import evaluate_parameters
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
import yaml

PACKAGE = 'ben_project11'
PREFIX = pathlib.Path(os.environ['BEN_PROJECT11_INSTALL_PREFIX'])
SHARE = PREFIX / 'share' / PACKAGE
NAMESPACE = 'ben'
NODE = 'bt_task_navigator'
PREFIXES = ['compute_path', 'follow_path', 'hover']
TIMEOUT = 30.0


def walk(entities):
    """Yield the entities and those inside GroupActions (included files are not loaded)."""
    for entity in entities:
        yield entity
        if isinstance(entity, GroupAction):
            yield from walk(entity.get_sub_entities())


def placeholder_substituted_params_file(context):
    """Return the params_file that nav2_bringup_launch.py hands to navigation_launch.py."""
    module = load_python_launch_file_as_module(str(SHARE / 'launch' / 'nav2_bringup_launch.py'))
    includes = [e for e in walk(module.generate_launch_description().entities)
                if isinstance(e, IncludeLaunchDescription)]
    assert len(includes) == 1, includes
    arguments = {perform_substitutions(context, normalize_to_list_of_substitutions(k)): v
                 for k, v in includes[0].launch_arguments}
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(arguments['params_file']))


def bt_task_navigator_parameters(context):
    """Evaluate the bt_task_navigator LifecycleNode's parameters in navigation_launch.py."""
    module = load_python_launch_file_as_module(str(SHARE / 'launch' / 'navigation_launch.py'))
    nodes = [e for e in walk(module.generate_launch_description().entities)
             if isinstance(e, LaunchNode) and e._Node__node_name == NODE]
    assert len(nodes) == 1, nodes
    parameters = nodes[0]._Node__parameters
    assert isinstance(parameters[-1], ParameterFile)
    evaluated = evaluate_parameters(context, parameters)
    # The ParameterFile deletes its evaluated file when it is collected: read it now.
    return (module.installed_nav2_bt_navigator_version(),
            evaluated[:-1], pathlib.Path(evaluated[-1]).read_text())


def stock_navigator_file(evaluated_text, out_dir):
    """Replace BEN's navigator plugin and BT library by the stock navigate_to_pose."""
    data = yaml.safe_load(evaluated_text)
    block = data[NAMESPACE][NODE]['ros__parameters']
    for key in block['navigators']:
        block.pop(key, None)
    block['navigators'] = ['navigate_to_pose']
    block['navigate_to_pose'] = {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'}
    block.pop('plugin_lib_names', None)
    path = out_dir / 'bt_task_navigator_stock.yaml'
    path.write_text(yaml.safe_dump({NAMESPACE: {NODE: {'ros__parameters': block}}}))
    return path


def call(node, client, request, process, log_path):
    """Call a service of the navigator; fail at once, with its log, if it has exited."""
    deadline = time.monotonic() + TIMEOUT
    while not client.wait_for_service(timeout_sec=0.2):
        assert process.poll() is None, \
            f'bt_navigator exited ({process.returncode}):\n{log_path.read_text()}'
        assert time.monotonic() < deadline, \
            f'{client.srv_name} not available:\n{log_path.read_text()}'
    future = client.call_async(request)
    while not future.done():
        rclpy.spin_until_future_complete(node, future, timeout_sec=0.2)
        assert future.done() or process.poll() is None, \
            f'bt_navigator exited ({process.returncode}):\n{log_path.read_text()}'
        assert time.monotonic() < deadline, \
            f'{client.srv_name} did not answer:\n{log_path.read_text()}'
    return future.result()


def test_bt_task_navigator_configures(tmp_path, monkeypatch):
    # nav2_common and launch_ros write their files with tempfile.
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    monkeypatch.setenv(
        'AMENT_PREFIX_PATH',
        os.pathsep.join([str(PREFIX), os.environ.get('AMENT_PREFIX_PATH', '')]))
    assert pathlib.Path(get_package_share_directory(PACKAGE)) == SHARE

    context = LaunchContext()
    context.launch_configurations.update({
        'namespace': NAMESPACE,
        'use_namespace': 'true',
        'params_file': str(SHARE / 'config' / 'nav2_params.yaml'),
        'autostart': 'true',
    })
    replaced = placeholder_substituted_params_file(context)
    assert '<robot_namespace>' not in pathlib.Path(replaced).read_text()
    context.launch_configurations['params_file'] = replaced

    version, extras, evaluated_text = bt_task_navigator_parameters(context)
    # In the launch file's order: the extras, then the parameter file.
    files = []
    for index, extra in enumerate(extras):
        path = tmp_path / f'extra_{index}.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': dict(
            (k, list(v) if isinstance(v, tuple) else v) for k, v in extra.items())}}))
        files.append(path)
    files.append(stock_navigator_file(evaluated_text, tmp_path))
    legacy = tuple(int(p) for p in version.split('.')[:2]) < (1, 4)

    executable = pathlib.Path(get_package_prefix('nav2_bt_navigator')) / 'lib' / \
        'nav2_bt_navigator' / 'bt_navigator'
    command = [str(executable), '--ros-args', '-r', f'__node:={NODE}',
               '-r', f'__ns:=/{NAMESPACE}']
    for path in files:
        command += ['--params-file', str(path)]
    log_path = tmp_path / 'bt_navigator.log'
    with open(log_path, 'w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    # From here on, every way out stops the navigator: rclpy's set-up is
    # inside the try, and each teardown step is guarded by the next.
    node = None
    initialized = False
    try:
        rclpy.init()
        initialized = True
        node = rclpy.create_node('test_bt_navigator_params')
        check_navigator(node, process, log_path, version, legacy)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if initialized:
                    rclpy.shutdown()
            finally:
                stop(process)


def check_navigator(node, process, log_path, version, legacy):
    """Configure the navigator and read its error-code parameters back."""
    prefix = f'/{NAMESPACE}/{NODE}'
    result = call(node, node.create_client(ChangeState, f'{prefix}/change_state'),
                  ChangeState.Request(transition=Transition(
                      id=Transition.TRANSITION_CONFIGURE)), process, log_path)
    log_text = log_path.read_text()
    assert result.success, f'configure failed (Nav2 {version}):\n{log_text}'
    state = call(node, node.create_client(GetState, f'{prefix}/get_state'),
                 GetState.Request(), process, log_path)
    assert state.current_state.id == State.PRIMARY_STATE_INACTIVE, log_text

    client = node.create_client(GetParameters, f'{prefix}/get_parameters')

    def string_array(name):
        # An undeclared name gets an empty answer.
        values = call(node, client, GetParameters.Request(names=[name]),
                      process, log_path).values
        if not values or values[0].type == ParameterType.PARAMETER_NOT_SET:
            return None
        return list(values[0].string_array_value)

    if legacy:
        assert string_array('error_code_names') == \
            [p + '_error_code' for p in PREFIXES], log_text
    else:
        assert string_array('error_code_names') is None, log_text
        assert string_array('error_code_name_prefixes') == PREFIXES, log_text
    print(f'Nav2 {version}:\n{log_text}')


def stop(process):
    """Stop the navigator: SIGINT, then SIGKILL after 10 s."""
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
