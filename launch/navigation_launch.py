# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


# Error codes of bt_task_navigator. config/nav2_params.yaml gives
# error_code_name_prefixes (Nav2 1.4 and later: Kilted 1.4, Lyrical 1.5 and
# Nav2's main branch, which Rolling builds), which refuse error_code_names
# at configure. Nav2 older than 1.4 (Jazzy 1.3) reads only
# error_code_names, and without it drops hover_error_code; it is given
# here for those versions only, ahead of the parameter file so that a
# file with its own error_code_names wins. Keep this list equal to the
# prefixes in the parameter file with '_error_code' appended
# (test/test_launch_and_config.py checks it).
LEGACY_ERROR_CODE_NAMES = [
    'compute_path_error_code',
    'follow_path_error_code',
    'hover_error_code',
]


# What each installed Nav2 package's version decides, for the error messages.
VERSION_PURPOSE = {
    'nav2_bt_navigator':
        'it decides whether bt_task_navigator gets error_code_names '
        '(Nav2 older than 1.4) or not',
    'nav2_smac_planner':
        'it decides whether planner_server gets GridBased.lookup_table_size '
        'overridden to 20 (nav2_smac_planner 1.5 and later) or not',
}


def nav2_version_tuple(version, package='nav2_bt_navigator'):
    """Return a version string such as '1.3.14' as a tuple of integers."""
    try:
        parts = tuple(int(part) for part in version.strip().split('.'))
    except (AttributeError, ValueError):
        parts = ()
    if len(parts) < 2:
        raise RuntimeError(
            f'ben_project11: cannot parse the {package} version {version!r}; '
            f'{VERSION_PURPOSE[package]}')
    return parts


def legacy_error_code_params(nav2_version):
    """Return the extra bt_task_navigator parameters for this Nav2 version."""
    if nav2_version_tuple(nav2_version) < (1, 4):
        return [{'error_code_names': list(LEGACY_ERROR_CODE_NAMES)}]
    return []


def installed_version(package):
    """Return the <version> of an installed package's package.xml."""
    try:
        share = get_package_share_directory(package)
        version = ET.parse(os.path.join(share, 'package.xml')).getroot().findtext('version')
    except Exception as error:
        raise RuntimeError(
            f'ben_project11: cannot read the version of the installed {package} '
            f'({error}); {VERSION_PURPOSE[package]}') from error
    if not version:
        raise RuntimeError(
            f'ben_project11: the installed {package} package.xml has no <version>; '
            f'{VERSION_PURPOSE[package]}')
    return version


def installed_nav2_bt_navigator_version():
    """Return the <version> of the installed nav2_bt_navigator's package.xml."""
    return installed_version('nav2_bt_navigator')


def installed_nav2_smac_planner_version():
    """Return the <version> of the installed nav2_smac_planner's package.xml."""
    return installed_version('nav2_smac_planner')


# lookup_table_size of SmacPlannerHybrid. Nav2's Smac planner refuses a
# value above 20 (m) from nav2_smac_planner 1.5 on ("lookup_table_size must
# be in [0.0, 20.0]", Nav2 d7bf2ac0 on main and 221cdb28 on lyrical; not on
# jazzy 1.3 or kilted 1.4), so planner_server does not configure with the
# 55.0 in config/nav2_params.yaml. From 1.5 on this launch file therefore
# overrides it to 20.0, with a parameter entry after the parameter file.
# - The override also replaces a custom params_file's
#   GridBased.lookup_table_size, and sets it when the file has none: on
#   those Nav2 versions change the value here, not in the parameter file.
# - It applies only to the planner id GridBased.
# - The package version cannot mark the limit exactly (main is 1.5.0 and
#   lyrical 1.5.2 both before and after it), so 1.5.0 to 1.5.2 are lowered
#   too; that only shrinks the heuristic table.
# - A backport of the limit to an older Nav2 line would not be detected.
LOOKUP_TABLE_SIZE_OVERRIDE = {'GridBased.lookup_table_size': 20.0}


def lookup_table_size_is_limited(smac_planner_version):
    """Return whether this nav2_smac_planner refuses lookup_table_size above 20."""
    return nav2_version_tuple(smac_planner_version, 'nav2_smac_planner') >= (1, 5)


def planner_server_params(smac_planner_version, configured_params):
    """Return planner_server's parameters for this nav2_smac_planner version."""
    if lookup_table_size_is_limited(smac_planner_version):
        return [configured_params, dict(LOOKUP_TABLE_SIZE_OVERRIDE)]
    return [configured_params]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'manda_coverage',
        'velocity_smoother',
        'collision_monitor',
        #'bt_navigator',
        'bt_task_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [] #('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )
    configured_params = ParameterFile(rewritten_params, allow_substs=True)

    # The parameter file, and after it on nav2_smac_planner 1.5 and later
    # GridBased.lookup_table_size 20.0 (see LOOKUP_TABLE_SIZE_OVERRIDE).
    planner_params = planner_server_params(
        installed_nav2_smac_planner_version(), configured_params)

    # Before the parameter file: launch_ros passes each entry as its own
    # --params-file, in this order, and a later file wins over an earlier
    # one for the same parameter. So the shipped file, which has no
    # error_code_names, gets the legacy names on Nav2 older than 1.4, and a
    # params_file that sets its own error_code_names keeps them when its
    # list is keyed by the node's name. (A list keyed '/**' loses when the
    # namespace is empty: SetParameter's -p use_sim_time makes '/**' the
    # first entry, so this list wins. BEN's namespace is 'ben'.)
    bt_task_navigator_params = legacy_error_code_params(
        installed_nav2_bt_navigator_version()) + [configured_params]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LifecycleNode(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=planner_params,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='manda_coverage',
                executable='manda_coverage_action_server',
                name='manda_coverage',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('soundings', 'sensors/mbes/soundings')],
                namespace="",
                emulate_tty=True
            ),
            # SetParameter(
            #     name = 'default_nav_to_pose_bt_xml',
            #     value = PathJoinSubstitution([
            #         FindPackageShare('nav2_bt_navigator'),
            #         'behavior_trees',
            #         'navigate_w_replanning_only_if_path_becomes_invalid.xml'
            #     ])
            # ),
            # SetParameter(
            #     name = 'default_nav_through_poses_bt_xml',
            #     value = PathJoinSubstitution([
            #         FindPackageShare('project11_navigation'),
            #         'behavior_trees',
            #         'navigate_through_poses.xml'
            #     ])
            # ),
            # LifecycleNode(
            #     package='nav2_bt_navigator',
            #     executable='bt_navigator',
            #     name='bt_navigator',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings,
            #     namespace=""
            # ),
            LifecycleNode(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_task_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=bt_task_navigator_params,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'piloting_mode/autonomous/cmd_vel')],
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace="",
                emulate_tty=True
            ),
            LifecycleNode(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace="",
                emulate_tty=True
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
                emulate_tty=True
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        # remappings=remappings + [('cmd_vel', 'piloting_mode/autonomous/cmd_vel')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=planner_params,
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_task_navigator',
                        parameters=bt_task_navigator_params,
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings
                        + [
                            ('cmd_vel', 'cmd_vel_nav'),
                            ('cmd_vel_smoothed', 'piloting_mode/autonomous/cmd_vel')
                        ],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings
                        + [('cmd_vel', 'piloting_mode/autonomous/cmd_vel')],
                    ),
                    ComposableNode(
                        package='opennav_docking',
                        plugin='opennav_docking::DockingServer',
                        name='docking_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
