"""Static checks of the installed launch and parameter files.

Run by CMake (ament_add_pytest_test) after colcon build, against the
install prefix it passes as BEN_PROJECT11_INSTALL_PREFIX. No ROS process
is started: launch descriptions are generated, not launched.

What is checked:
- the Python launch files load and declare a fixed list of arguments;
- every parameter file they name parses, each node block holding a
  ros__parameters mapping;
- every package a launch file names (package=, FindPackageShare,
  get_package_share_directory, imports) and every package a parameter file
  names (plugin: and plugin_lib_names entries) is declared in package.xml;
- the error-code settings of bt_task_navigator: error_code_names is in no
  parameter file, error_code_name_prefixes holds hover,
  legacy_error_code_params gives the old error_code_names for Nav2 older
  than 1.4 and nothing from 1.4 on, an unparsable version is an error,
  and both bt_task_navigator descriptions (the lifecycle node and the
  composable node) carry its result before the parameter file, so that
  a parameter file's own error_code_names wins (launch_ros passes the
  entries in order and the later one wins);
- lookup_table_size of planner_server: the shipped file keeps 55.0 (what
  Jazzy reads); lookup_table_size_is_limited is true from
  nav2_smac_planner 1.5 on and false before, an unparsable or unreadable
  version is an error; planner_server_params gives exactly the parameter
  file before 1.5 and the file followed by GridBased.lookup_table_size
  20.0 from 1.5 on, and both planner_server descriptions carry it (the
  override after the file, so that it wins).

The ROS 1 launch files (launch/*.launch) are not checked: they are XML for
ROS 1, outside the launch/*.py set by their extension, and cannot run on
ROS 2.
"""

import ast
import os
import pathlib
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.launch_description_sources.python_launch_file_utilities import (
    load_python_launch_file_as_module,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node as LaunchNode
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

PACKAGE = 'ben_project11'
PREFIX = pathlib.Path(os.environ['BEN_PROJECT11_INSTALL_PREFIX'])
SHARE = PREFIX / 'share' / PACKAGE

LAUNCH_ARGUMENTS = {
    'ben_core_launch.py': [
        'namespace', 'enable_bridge', 'tf_prefix', 'base_frame', 'map_frame',
        'odom_frame', 'is_simulator', 'launch_robot_state_publisher', 'params_file',
    ],
    'nav2_bringup_launch.py': [
        'namespace', 'use_namespace', 'use_sim_time', 'params_file', 'autostart',
        'use_composition', 'use_respawn', 'log_level',
    ],
    'navigation_launch.py': [
        'namespace', 'use_sim_time', 'params_file', 'autostart', 'use_composition',
        'container_name', 'use_respawn', 'log_level',
    ],
}
PARAMETER_FILES = {'nav2_params.yaml', 'ben.yaml', 'ben_sim.yaml'}

# plugin_lib_names entries (behaviour-tree plugin libraries) and their packages.
BT_LIBRARY_PACKAGES = {
    'marine_nav_behavior_tree_bt_plugins': 'marine_nav_behavior_tree',
}

# Python modules whose rosdep key is not their name.
IMPORT_PACKAGES = {
    'yaml': 'python3-yaml',
}

NAVIGATOR = 'bt_task_navigator'


@pytest.fixture(autouse=True)
def this_install_first(monkeypatch):
    # nav2_bringup_launch.py looks up this package's share directory when it
    # is loaded: make it find the install under test, not another copy.
    monkeypatch.setenv(
        'AMENT_PREFIX_PATH',
        os.pathsep.join([str(PREFIX), os.environ.get('AMENT_PREFIX_PATH', '')]))
    assert pathlib.Path(get_package_share_directory(PACKAGE)) == SHARE


def launch_path(name):
    return SHARE / 'launch' / name


def load(name):
    return load_python_launch_file_as_module(str(launch_path(name)))


def walk(entities):
    """Yield entities, those inside GroupActions and composable node descriptions."""
    for entity in entities:
        yield entity
        if isinstance(entity, GroupAction):
            yield from walk(entity.get_sub_entities())
        if isinstance(entity, LoadComposableNodes):
            yield from entity._LoadComposableNodes__composable_node_descriptions


def syntax_tree(name):
    return ast.parse(launch_path(name).read_text(), filename=name)


def yaml_files():
    return {name: yaml.safe_load((SHARE / 'config' / name).read_text())
            for name in sorted(PARAMETER_FILES)}


def all_keys_and_values(data):
    if isinstance(data, dict):
        for key, value in data.items():
            yield key, value
            yield from all_keys_and_values(value)
    elif isinstance(data, list):
        for item in data:
            yield from all_keys_and_values(item)


def declared_packages():
    root = ET.parse(SHARE / 'package.xml').getroot()
    return {e.text.strip() for e in root if e.tag in ('depend', 'exec_depend')}


def nav2_params():
    return yaml_files()['nav2_params.yaml'][NAVIGATOR]['ros__parameters']


# Launch files


def test_python_launch_files_are_the_checked_set():
    assert sorted(p.name for p in (SHARE / 'launch').glob('*.py')) == sorted(LAUNCH_ARGUMENTS)


@pytest.mark.parametrize('name', sorted(LAUNCH_ARGUMENTS))
def test_launch_file_loads_with_its_arguments(name):
    description = load(name).generate_launch_description()
    declared = [e.name for e in description.entities if isinstance(e, DeclareLaunchArgument)]
    assert declared == LAUNCH_ARGUMENTS[name]


# Parameter files


def test_launch_files_name_the_parameter_files():
    named = set()
    for name in LAUNCH_ARGUMENTS:
        for node in ast.walk(syntax_tree(name)):
            if isinstance(node, ast.Constant) and isinstance(node.value, str) and \
                    node.value.endswith('.yaml'):
                named.add(node.value)
    assert named == PARAMETER_FILES


@pytest.mark.parametrize('name', sorted(PARAMETER_FILES))
def test_parameter_file_parses_into_node_blocks(name):
    data = yaml.safe_load((SHARE / 'config' / name).read_text())
    assert isinstance(data, dict) and data, name

    def reaches_parameters(block, depth):
        if not isinstance(block, dict) or depth > 2:
            return False
        if 'ros__parameters' in block:
            return isinstance(block['ros__parameters'], dict)
        # An empty block reaches nothing (all() of nothing would be true).
        return bool(block) and \
            all(reaches_parameters(value, depth + 1) for value in block.values())

    for key, block in data.items():
        assert reaches_parameters(block, 0), f'{name}: {key}'


# Manifest


def packages_named_by_launch_files():
    named = set()
    stdlib = set(sys.stdlib_module_names)
    for name in LAUNCH_ARGUMENTS:
        for node in ast.walk(syntax_tree(name)):
            if isinstance(node, ast.Call):
                function = getattr(node.func, 'id', getattr(node.func, 'attr', None))
                if function in ('FindPackageShare', 'get_package_share_directory',
                                'installed_version') and \
                        node.args and isinstance(node.args[0], ast.Constant):
                    named.add(node.args[0].value)
                for keyword in node.keywords:
                    if keyword.arg == 'package' and isinstance(keyword.value, ast.Constant):
                        named.add(keyword.value.value)
            elif isinstance(node, ast.Import):
                named.update(alias.name.split('.')[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                named.add(node.module.split('.')[0])
    return {IMPORT_PACKAGES.get(n, n) for n in named if n not in stdlib}


def packages_named_by_parameter_files():
    named = set()
    for name, data in yaml_files().items():
        for key, value in all_keys_and_values(data):
            if key == 'plugin' and isinstance(value, str):
                named.add(value.split('::')[0])
            if key == 'plugin_lib_names':
                for library in value:
                    assert library in BT_LIBRARY_PACKAGES, \
                        f'{name}: add the package of {library} to BT_LIBRARY_PACKAGES'
                    named.add(BT_LIBRARY_PACKAGES[library])
    return named


def test_every_named_package_is_declared():
    named = packages_named_by_launch_files() | packages_named_by_parameter_files()
    assert {'nav2_bt_navigator', 'nav2_smac_planner', 's57_layer'} <= named
    missing = sorted(named - declared_packages() - {PACKAGE})
    assert not missing, f'not declared in package.xml: {missing}'


# Error codes of bt_task_navigator


def test_error_code_names_is_in_no_parameter_file():
    for name, data in yaml_files().items():
        assert 'error_code_names' not in [k for k, _ in all_keys_and_values(data)], name


def test_error_code_name_prefixes_hold_hover():
    assert 'hover' in nav2_params()['error_code_name_prefixes']


@pytest.mark.parametrize('version', ['1.3.0', '1.3.14', '1.3'])
def test_legacy_error_code_params_before_1_4(version):
    expected = [p + '_error_code' for p in nav2_params()['error_code_name_prefixes']]
    assert load('navigation_launch.py').legacy_error_code_params(version) == \
        [{'error_code_names': expected}]


@pytest.mark.parametrize('version', ['1.4.0', '1.4.2', '1.5.0', '2.0.0'])
def test_legacy_error_code_params_from_1_4(version):
    assert load('navigation_launch.py').legacy_error_code_params(version) == []


@pytest.mark.parametrize('version', ['', 'abc', '1', '1.4.0rc1', None])
def test_unparsable_version_is_an_error(version):
    with pytest.raises(RuntimeError, match='nav2_bt_navigator version'):
        load('navigation_launch.py').legacy_error_code_params(version)


def test_installed_nav2_version_is_read():
    module = load('navigation_launch.py')
    module.nav2_version_tuple(module.installed_nav2_bt_navigator_version())
    module.nav2_version_tuple(
        module.installed_nav2_smac_planner_version(), 'nav2_smac_planner')


def node_parameters(description, name):
    """Return {'LifecycleNode': parameters, 'ComposableNode': parameters} of a node name."""
    context = LaunchContext()
    found = {}
    for entity in walk(description.entities):
        if isinstance(entity, LaunchNode) and entity._Node__node_name == name:
            found.setdefault('LifecycleNode', []).append(entity._Node__parameters)
        elif isinstance(entity, ComposableNode) and \
                perform_substitutions(context, entity.node_name) == name:
            found.setdefault('ComposableNode', []).append(entity.parameters)
    assert sorted(found) == ['ComposableNode', 'LifecycleNode'], found
    assert all(len(descriptions) == 1 for descriptions in found.values()), found
    return {kind: descriptions[0] for kind, descriptions in found.items()}


@pytest.mark.parametrize('version', ['1.3.14', '1.5.0'])
def test_both_bt_task_navigators_get_the_legacy_parameters(version):
    module = load('navigation_launch.py')
    module.installed_nav2_bt_navigator_version = lambda: version
    expected = [dict(p) for p in module.legacy_error_code_params(version)]
    context = LaunchContext()

    description = module.generate_launch_description()
    for kind, parameters in node_parameters(description, NAVIGATOR).items():
        # The parameter file last, the helper's result (if any) before it.
        assert isinstance(parameters[-1], ParameterFile), kind
        before = [{key: list(value) for key, value in p.items()}
                  for p in evaluate_parameters(context, parameters[:-1])]
        assert before == expected, kind


# lookup_table_size of planner_server

PLANNER = 'planner_server'
HYBRID = 'nav2_smac_planner::SmacPlannerHybrid'


def test_shipped_lookup_table_size_stays_55_for_jazzy():
    block = yaml_files()['nav2_params.yaml'][PLANNER]['ros__parameters']
    assert block['planner_plugins'] == ['GridBased']
    assert block['GridBased']['plugin'] == HYBRID
    assert block['GridBased']['lookup_table_size'] == 55.0


@pytest.mark.parametrize('version', ['1.5.0', '1.5.2', '1.5.3', '1.6.0', '2.0.0'])
def test_lookup_table_size_is_limited_from_1_5(version):
    assert load('navigation_launch.py').lookup_table_size_is_limited(version) is True


@pytest.mark.parametrize('version', ['1.3.0', '1.3.14', '1.3', '1.4.0', '1.4.2'])
def test_lookup_table_size_is_not_limited_before_1_5(version):
    assert load('navigation_launch.py').lookup_table_size_is_limited(version) is False


@pytest.mark.parametrize('version', ['', 'abc', '1', '1.5.0rc1', None])
def test_unparsable_smac_planner_version_is_an_error(version):
    with pytest.raises(RuntimeError, match='nav2_smac_planner version'):
        load('navigation_launch.py').lookup_table_size_is_limited(version)


def test_unparsable_installed_smac_planner_version_stops_the_launch_file():
    module = load('navigation_launch.py')
    module.installed_nav2_smac_planner_version = lambda: 'abc'
    with pytest.raises(RuntimeError, match='nav2_smac_planner version'):
        module.generate_launch_description()


def test_unreadable_smac_planner_version_is_an_error(tmp_path):
    module = load('navigation_launch.py')

    def missing(package):
        raise LookupError(f'no {package}')
    module.get_package_share_directory = missing
    with pytest.raises(RuntimeError, match='cannot read the version of the installed '
                                           'nav2_smac_planner'):
        module.installed_nav2_smac_planner_version()

    (tmp_path / 'package.xml').write_text('<package><name>x</name></package>')
    module.get_package_share_directory = lambda package: str(tmp_path)
    with pytest.raises(RuntimeError, match='nav2_smac_planner package.xml has no <version>'):
        module.installed_nav2_smac_planner_version()


@pytest.mark.parametrize('version', ['1.3.14', '1.4.2'])
def test_planner_server_params_before_1_5_is_the_file_alone(version):
    configured_params = object()
    assert load('navigation_launch.py').planner_server_params(version, configured_params) \
        == [configured_params]
    [entry] = load('navigation_launch.py').planner_server_params(version, configured_params)
    assert entry is configured_params


@pytest.mark.parametrize('version', ['1.5.0', '1.5.3', '2.0.0'])
def test_planner_server_params_from_1_5_override_after_the_file(version):
    configured_params = object()
    entries = load('navigation_launch.py').planner_server_params(version, configured_params)
    assert entries[0] is configured_params
    assert entries[1:] == [{'GridBased.lookup_table_size': 20.0}]


def test_unparsable_smac_planner_version_stops_planner_server_params():
    with pytest.raises(RuntimeError, match='nav2_smac_planner version'):
        load('navigation_launch.py').planner_server_params('abc', object())


@pytest.mark.parametrize('version', ['1.3.14', '1.4.2', '1.5.0'])
def test_both_planner_servers_get_the_parameter_file_then_the_override(version):
    module = load('navigation_launch.py')
    module.installed_nav2_smac_planner_version = lambda: version
    expected = [{'GridBased.lookup_table_size': 20.0}] \
        if module.lookup_table_size_is_limited(version) else []
    context = LaunchContext()
    description = module.generate_launch_description()
    [controller] = node_parameters(description, 'controller_server')['LifecycleNode']
    for kind, parameters in node_parameters(description, PLANNER).items():
        # First a ParameterFile of the very RewrittenYaml every other node
        # gets (launch_ros copies the ParameterFile itself when it normalizes
        # the list, so the RewrittenYaml is what can be compared by
        # identity), then the override, if any: the later entry wins.
        assert isinstance(parameters[0], ParameterFile), kind
        [source] = parameters[0].param_file
        assert source is controller.param_file[0], kind
        assert type(source).__name__ == 'RewrittenYaml', kind
        assert list(evaluate_parameters(context, parameters[1:])) == expected, kind
