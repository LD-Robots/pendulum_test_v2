"""PVT / MIT mode 5 — drive-side impedance control.

  ros2 launch pendulum_pvt_control pvt.launch.py            # real (default)
  ros2 launch pendulum_pvt_control pvt.launch.py use_sim:=true

Real: the myActuator X6 runs tau = Kp*(q_d - q) + Kd*(qd_d - qd) + tau_ff in
firmware; the controller streams position/velocity/effort/kp/kd (RxPDO 0x1601).
Sim: the same controller replicates that law in software and writes effort only;
Gazebo bringup is reused from the pendulum_gazebo package.

Stream setpoints to /pendulum_pvt_controller/setpoint
(trajectory_msgs/JointTrajectoryPoint). /pendulum_pvt_controller/hold snapshots
the current position; /pendulum_pvt_controller/free drops to zero drive torque.
Ramp Kp/Kd live with `ros2 param set /pendulum_pvt_controller Kp|Kd <value>`.
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
    pkg_share  = get_package_share_directory('pendulum_pvt_control')
    desc_share = get_package_share_directory('pendulum_description')

    controllers_yaml  = os.path.join(pkg_share, 'config', 'controllers_pvt.yaml')
    gains_yaml        = os.path.join(pkg_share, 'config', 'pvt_gains.yaml')
    gains_sim_yaml    = os.path.join(pkg_share, 'config', 'pvt_gains_sim.yaml')
    slave_yaml        = os.path.join(pkg_share, 'config', 'ethercat', 'icube_x6_drive_pvt.yaml')
    drive_status_yaml = os.path.join(pkg_share, 'config', 'drive_status_broadcaster.yaml')

    use_sim = LaunchConfiguration('use_sim')

    real_xacro = os.path.join(desc_share, 'urdf', 'pendulum_ethercat.urdf.xacro')
    sim_xacro  = os.path.join(desc_share, 'urdf', 'pendulum.urdf.xacro')

    real_robot_description = ParameterValue(
        Command([
            'xacro ', real_xacro,
            ' slave_config:=', slave_yaml,
            ' mode_of_operation:=5',
            ' pvt_mode:=true',
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

    # Reusable Gazebo bringup — resource paths, custom world, robot spawn from
    # /robot_description, /clock bridge. rsp_sim publishes /robot_description;
    # sim_bringup deliberately does not start robot_state_publisher.
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pendulum_gazebo'), 'launch', 'sim_bringup.launch.py'])
        ]),
        condition=IfCondition(use_sim),
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    # Real: drive_side_pd defaults to true in pvt_gains.yaml — the X6 runs the
    # PD law and the controller claims position/velocity/effort/kp/kd.
    pvt_spawner_real = Node(
        package='controller_manager', executable='spawner',
        condition=UnlessCondition(use_sim),
        arguments=['pendulum_pvt_controller', '--param-file', gains_yaml],
        output='screen',
    )
    # Sim: the second --param-file overrides drive_side_pd to false — the
    # controller runs the PD law in software and claims only effort.
    pvt_spawner_sim = Node(
        package='controller_manager', executable='spawner',
        condition=IfCondition(use_sim),
        arguments=['pendulum_pvt_controller',
                   '--param-file', gains_yaml,
                   '--param-file', gains_sim_yaml],
        output='screen',
    )

    # Drive telemetry broadcaster — real hardware only (sim has no temp /
    # voltage / error_code interfaces). Republishes error_code, motor/drive
    # temperature and bus voltage as Float64 topics + /diagnostics.
    drive_status_spawner = Node(
        package='controller_manager', executable='spawner',
        condition=UnlessCondition(use_sim),
        arguments=['drive_status_broadcaster', '--param-file', drive_status_yaml],
        output='screen',
    )

    pvt_real_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[pvt_spawner_real])
    )
    pvt_sim_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[pvt_spawner_sim])
    )
    drive_status_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[drive_status_spawner])
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        GroupAction([rsp_real, ros2_control_node]),
        GroupAction([rsp_sim, sim_bringup]),
        jsb_spawner,
        pvt_real_after_jsb,
        pvt_sim_after_jsb,
        drive_status_after_jsb,
    ])
