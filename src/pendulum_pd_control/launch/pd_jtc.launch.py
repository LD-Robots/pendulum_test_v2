"""Variant 2 — stock joint_trajectory_controller with effort + gains.

  ros2 launch pendulum_pd_control pd_jtc.launch.py            # real (default)
  ros2 launch pendulum_pd_control pd_jtc.launch.py use_sim:=true

Send setpoints via /pendulum_jtc/joint_trajectory or the
/pendulum_jtc/follow_joint_trajectory action.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share  = get_package_share_directory('pendulum_pd_control')
    desc_share = get_package_share_directory('pendulum_description')

    controllers_yaml  = os.path.join(pkg_share, 'config', 'controllers_jtc.yaml')
    drive_status_yaml = os.path.join(pkg_share, 'config', 'drive_status_broadcaster.yaml')
    slave_yaml        = os.path.join(pkg_share, 'config', 'ethercat', 'icube_x6_drive.yaml')

    use_sim = LaunchConfiguration('use_sim')

    real_xacro = os.path.join(desc_share, 'urdf', 'pendulum_ethercat.urdf.xacro')
    sim_xacro  = os.path.join(desc_share, 'urdf', 'pendulum.urdf.xacro')

    real_robot_description = ParameterValue(
        Command([
            'xacro ', real_xacro,
            ' slave_config:=', slave_yaml,
            ' mode_of_operation:=10',
            ' pvt_mode:=false',
        ]),
        value_type=str,
    )
    sim_robot_description = ParameterValue(
        Command([
            'xacro ', sim_xacro,
            ' controllers_yaml:=', controllers_yaml,
        ]),
        value_type=str,
    )

    rsp_real = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        condition=UnlessCondition(use_sim),
        parameters=[{'robot_description': real_robot_description, 'use_sim_time': False}],
    )
    rsp_sim = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        condition=IfCondition(use_sim),
        parameters=[{'robot_description': sim_robot_description, 'use_sim_time': True}],
    )

    ros2_control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        condition=UnlessCondition(use_sim),
        parameters=[{'robot_description': real_robot_description}, controllers_yaml],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        condition=IfCondition(use_sim),
    )
    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        condition=IfCondition(use_sim),
        arguments=['-name', 'pendulum', '-topic', 'robot_description'],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    jtc_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['pendulum_jtc'],
        output='screen',
    )
    # Drive telemetry broadcaster — real hardware only (sim has no temp/voltage
    # interfaces). Runs alongside the JTC.
    drive_status_spawner = Node(
        package='controller_manager', executable='spawner',
        condition=UnlessCondition(use_sim),
        arguments=['drive_status_broadcaster', '--param-file', drive_status_yaml],
        output='screen',
    )

    jtc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[jtc_spawner])
    )
    drive_status_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[drive_status_spawner])
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        GroupAction([rsp_real, ros2_control_node]),
        GroupAction([rsp_sim, gz_sim, spawn_entity]),
        jsb_spawner,
        jtc_after_jsb,
        drive_status_after_jsb,
    ])
