from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParametersFromFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  namespace = LaunchConfiguration('namespace')
  enable_bridge = LaunchConfiguration('enable_bridge')

  tf_prefix = LaunchConfiguration('tf_prefix')
  base_frame = LaunchConfiguration('base_frame')
  map_frame = LaunchConfiguration('map_frame')
  odom_frame = LaunchConfiguration('odom_frame')

  is_simulator = LaunchConfiguration('is_simulator')

  namespace_arg = DeclareLaunchArgument(
    "namespace", default_value=TextSubstitution(text="ben")
  )
  enable_bridge_arg = DeclareLaunchArgument(
    "enable_bridge", default_value=TextSubstitution(text="true")
  )

  tf_prefix_arg = DeclareLaunchArgument(
    "tf_prefix", default_value=namespace
  )
  base_frame_arg = DeclareLaunchArgument(
    "base_frame", default_value=PathJoinSubstitution([tf_prefix, 'base_link'])
  )
  map_frame_arg = DeclareLaunchArgument(
    "map_frame", default_value=PathJoinSubstitution([tf_prefix, 'map'])
  )
  odom_frame_arg = DeclareLaunchArgument(
    "odom_frame", default_value=PathJoinSubstitution([tf_prefix, 'odom'])
  )

  is_simulator_arg = DeclareLaunchArgument(
    "is_simulator", default_value=TextSubstitution(text="false")
  )


  launch_publish_state_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ben_description'),
        'launch',
        'publish_state_launch.py'
      ])
    )
  )


  # <remap from="local_costmap" to="sensors/lidar/lidar_costmap/costmap/costmap"/>
  load_ben_parameters = SetParametersFromFile(
    filename=PathJoinSubstitution([
      FindPackageShare('ben_project11'),
      'config',
      'ben.yaml'
    ])
  )

  load_sim_parameters = SetParametersFromFile(
    filename=PathJoinSubstitution([
      FindPackageShare('ben_project11'),
      'config',
      'ben_sim.yaml'
    ]),
    condition=IfCondition(is_simulator)
  )

  launch_robot_core_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('project11'),
        'launch',
        'robot_core_launch.py'
      ])
    ),
    launch_arguments={
      'namespace': namespace,
      'enable_bridge': enable_bridge
    }.items()
  )


  remappings = []#('/tf', 'tf'), ('/tf_static', 'tf_static')]

  # mru_transform Provides tf2 transforms from multiple gps and motion sensor sources.
  mru_node = Node(
    package='mru_transform',
    executable='mru_transform_node',
    name='mru_transform',
    parameters=[
      {'base_frame': base_frame},
      {'map_frame': map_frame},
      {'odom_frame': odom_frame}
    ],
    remappings=remappings,
  )


  launch_nav2_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('ben_project11'),
        'launch',
        'nav2_bringup_launch.py'
      ])
    ),
    launch_arguments={
      'namespace': namespace,
      'use_namespace': 'true',
      'use_composition': 'False',
      'use_respawn': 'True',
    }.items()
  )

  namespace_group = GroupAction(
    actions=[
      PushROSNamespace(namespace),
      load_ben_parameters,
      load_sim_parameters,
      launch_robot_core_include,
      mru_node,
    ]
  )

  return LaunchDescription([
    namespace_arg,
    enable_bridge_arg,
    tf_prefix_arg,
    base_frame_arg,
    map_frame_arg,
    odom_frame_arg,
    is_simulator_arg,
    launch_publish_state_include,
    namespace_group,
    launch_nav2_include
  ])
