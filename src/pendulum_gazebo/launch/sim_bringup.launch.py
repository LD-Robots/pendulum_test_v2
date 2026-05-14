"""Reusable Gazebo Sim bringup for the pendulum testbed.

Sets GZ_SIM_RESOURCE_PATH so package:// meshes (rewritten to model:// by the
URDF->SDF conversion) resolve, starts Gazebo with the shipped custom world,
spawns the robot from the /robot_description topic, and bridges the gz /clock
topic to ROS so the in-process gz_ros2_control controller_manager gets sim time.

Included by the pendulum_pd_control sim launches; not meant to be run directly
(it does not start robot_state_publisher — the caller publishes /robot_description).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = get_package_share_directory('pendulum_description')
    gz_share = get_package_share_directory('pendulum_gazebo')

    default_world = os.path.join(gz_share, 'worlds', 'pendulum.sdf')

    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Absolute path to the Gazebo world SDF.')
    entity_arg = DeclareLaunchArgument(
        'entity_name', default_value='pendulum',
        description='Name of the spawned robot entity.')
    robot_desc_topic_arg = DeclareLaunchArgument(
        'robot_description_topic', default_value='robot_description',
        description='Topic carrying the robot_description to spawn from.')

    world = LaunchConfiguration('world')
    entity = LaunchConfiguration('entity_name')
    desc_topic = LaunchConfiguration('robot_description_topic')

    # Bug A fix: ros_gz_sim's URDF->SDF conversion rewrites package:// mesh URIs
    # to model://, which Gazebo resolves by searching GZ_SIM_RESOURCE_PATH.
    # dirname(desc_share) is the share/ dir that contains pendulum_description/.
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', entity, '-topic', desc_topic],
        output='screen',
    )

    # Bug B fix: bridge Gazebo's gz-transport /clock onto the ROS graph so the
    # gz_ros2_control controller_manager (running in the Gazebo process) gets
    # sim time instead of warning "No clock received".
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        entity_arg,
        robot_desc_topic_arg,
        gz_resource_path,
        gz_sim,
        spawn_entity,
        clock_bridge,
    ])
