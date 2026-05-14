"""Launch URDF visualization in RViz2 with joint_state_publisher_gui."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('pendulum_description')
    display_share = get_package_share_directory('pendulum_display')

    xacro_file = os.path.join(description_share, 'urdf', 'pendulum.urdf.xacro')
    rviz_config = os.path.join(display_share, 'rviz', 'display.rviz')

    # Wrap as string — mesh "file://" paths break YAML parsing otherwise.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
