"""Full RL-policy deployment for the X6 pendulum (PVT Mode 5).

Brings up everything in one command:
  - the PVT controller stack (via pendulum_pvt_control/pvt.launch.py,
    launched UNMODIFIED — it comes up with its own pvt_gains.yaml);
  - once the controller is up, this launch pushes pvt_gains_policy.yaml onto
    the running /pendulum_pvt_controller so the drive Kp/Kd (and ff flags)
    match the values the policy was trained with;
  - the ONNX inference node that streams setpoints to the controller.

The controller re-reads its parameters every update() tick, so the param
load takes effect on the next cycle — no controller restart needed.

Usage:
    ros2 launch pendulum_pvt_policy policy.launch.py \\
        onnx_path:=<workspace>/models/pvt_v1/policy.onnx          # real
    ros2 launch pendulum_pvt_policy policy.launch.py \\
        onnx_path:=.../policy.onnx use_sim:=true                  # Gazebo

Send a target the policy will track:
    ros2 topic pub --once /pendulum/target std_msgs/msg/Float64 "{data: 1.5}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    policy_share = get_package_share_directory('pendulum_pvt_policy')
    pvt_control_share = get_package_share_directory('pendulum_pvt_control')

    params_file = os.path.join(policy_share, 'config', 'policy.yaml')
    # Gains matched to the trained policy — lives in THIS package because the
    # gains are a property of the trained model, not of the controller.
    policy_gains = os.path.join(policy_share, 'config', 'pvt_gains_policy.yaml')
    pvt_launch = os.path.join(pvt_control_share, 'launch', 'pvt.launch.py')
    # Shared safety limits — the supervisor itself comes up via pvt.launch.py
    # (included below); the policy node only needs the safety.* limit values.
    safety_yaml = os.path.join(
        get_package_share_directory('pendulum_safety'),
        'config', 'safety_limits.yaml')

    # --- ONNX Runtime shared lib path (safety net for the RPATH) ---
    _ws_candidates = [
        os.path.normpath(os.path.join(policy_share, '..', '..', '..', '..',
                                      'third_party', 'onnxruntime', 'lib')),
        os.path.normpath(os.path.join(policy_share, '..', '..', '..',
                                      'third_party', 'onnxruntime', 'lib')),
    ]
    onnx_lib = next((p for p in _ws_candidates if os.path.isdir(p)), None)
    if onnx_lib:
        ld = os.environ.get('LD_LIBRARY_PATH', '')
        if onnx_lib not in ld:
            os.environ['LD_LIBRARY_PATH'] = onnx_lib + (':' + ld if ld else '')

    onnx_path_arg = DeclareLaunchArgument(
        'onnx_path',
        description='Absolute path to the trained policy.onnx.')
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='true → Gazebo sim, false → real EtherCAT hardware.')
    target_pos_arg = DeclareLaunchArgument(
        'target_pos', default_value='0.0',
        description='Initial /pendulum/target in rad.')
    policy_enabled_arg = DeclareLaunchArgument(
        'policy_enabled', default_value='true',
        description='Run inference (true) or pass user_target through (false).')
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='true → policy node logs the observation vector each '
                    'inference tick. Default off (quiet).')

    # PVT controller stack — launched untouched (its own pvt_gains.yaml).
    pvt_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pvt_launch),
        launch_arguments={'use_sim': LaunchConfiguration('use_sim')}.items(),
    )

    # Once /pendulum_pvt_controller exists, load the policy-matched gains onto
    # it. Polls until the controller's parameters are queryable, then applies
    # pvt_gains_policy.yaml. The controller picks them up on its next update().
    apply_policy_gains = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ros2 param list /pendulum_pvt_controller >/dev/null 2>&1; '
            'do sleep 0.5; done; '
            'ros2 param load /pendulum_pvt_controller ' + policy_gains + '; '
            'echo "[policy.launch] applied pvt_gains_policy.yaml to '
            '/pendulum_pvt_controller"',
        ],
        output='screen',
    )

    policy_node = Node(
        package='pendulum_pvt_policy',
        executable='pendulum_pvt_policy',
        name='pendulum_pvt_policy',
        output='screen',
        parameters=[
            params_file,
            safety_yaml,
            {
                'onnx_path': LaunchConfiguration('onnx_path'),
                'target_pos': LaunchConfiguration('target_pos'),
                'policy_enabled': LaunchConfiguration('policy_enabled'),
                'debug': LaunchConfiguration('debug'),
            },
        ],
    )

    return LaunchDescription([
        onnx_path_arg,
        use_sim_arg,
        target_pos_arg,
        policy_enabled_arg,
        debug_arg,
        pvt_stack,
        apply_policy_gains,
        policy_node,
    ])
