"""
njord_sim.launch.py — 2D simulator with the full Njord control pipeline.

Topic flow:
  mission_manager
    → /fromLL (sim_bridge)            GPS → metres
    → NavigateToPose action (simple_navigator)
    → /njord/cmd_vel
    → nav_to_pid  (/cmd_vel remapped to /njord/cmd_vel)
    → /control/setpoint
    → pid_controller  (reads /imu/data from sim_bridge)
    → /control/effort
    → sim_bridge  (republishes as /cmd_vel)
    → simulator_node

Run:
  source /opt/ros/jazzy/setup.bash
  ros2 launch asket_ec_sim2d njord_sim.launch.py

Prerequisites (one-time):
  sudo apt install ros-jazzy-robot-localization ros-jazzy-nav2-msgs
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('asket_ec_sim2d')
    rviz_config = os.path.join(pkg, 'config', 'sim2d.rviz')
    buoys_file  = os.path.join(pkg, 'config', 'buoys.yaml')

    return LaunchDescription([

        # ── Core simulator ──────────────────────────────────────────
        Node(
            package='asket_ec_sim2d',
            executable='simulator_node',
            name='sim2d_simulator',
            output='screen',
        ),
        Node(
            package='asket_ec_sim2d',
            executable='buoy_simulator_node',
            name='buoy_simulator',
            output='screen',
            parameters=[{'buoys_file': buoys_file}],
        ),

        # ── Bridge (fromLL + IMU + GPS + effort→cmd_vel) ────────────
        Node(
            package='asket_ec_sim2d',
            executable='sim_bridge_node',
            name='sim_bridge',
            output='screen',
        ),

        # ── Njord path planning ─────────────────────────────────────
        # simple_navigator: implements NavigateToPose, publishes /njord/cmd_vel
        Node(
            package='asket_ec_sim2d',
            executable='simple_navigator_node',
            name='simple_navigator',
            output='screen',
        ),
        Node(
            package='asket_ec_sim2d',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
        ),

        # ── Njord control chain ─────────────────────────────────────
        # nav_to_pid reads /njord/cmd_vel (remapped from its default /cmd_vel)
        Node(
            package='asket_ec_sim2d',
            executable='nav_to_pid',
            name='nav_to_pid',
            output='screen',
            remappings=[('/cmd_vel', '/njord/cmd_vel')],
        ),
        # pid_controller reads /control/setpoint + /imu/data → /control/effort
        Node(
            package='asket_ec_sim2d',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
        ),

        # ── Visualisation ───────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
