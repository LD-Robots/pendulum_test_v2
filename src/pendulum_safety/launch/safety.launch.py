"""pendulum_safety — launches the safety supervisor.

Standalone:
  ros2 launch pendulum_safety safety.launch.py                # real hardware
  ros2 launch pendulum_safety safety.launch.py use_sim:=true  # simulation

Also included by pvt.launch.py / pd_custom.launch.py (which forward use_sim).
Simulation has no temperature interfaces, so temperature monitoring is disabled
there (derived from use_sim).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pendulum_safety')
    limits_yaml = os.path.join(pkg_share, 'config', 'safety_limits.yaml')

    use_sim = LaunchConfiguration('use_sim')

    supervisor = Node(
        package='pendulum_safety',
        executable='pendulum_safety_supervisor',
        name='pendulum_safety_supervisor',
        output='screen',
        parameters=[
            limits_yaml,
            {
                'use_sim_time': use_sim,
                # Simulation has no temperature interfaces.
                'monitor_temperature': PythonExpression(
                    ["'", use_sim, "' == 'false'"]),
                # effort_topic stays /joint_states (from safety_limits.yaml) —
                # it carries the drive's actual torque (0x6077) on real
                # hardware and the gz effort in sim, on every launch path.
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='true in Gazebo: disables temperature monitoring.'),
        supervisor,
    ])
