"""
beatnaut_full.launch.py — Stack complète : Sim2D + contrôleur différentiel

=========================================================
CHANGEMENT : GAZEBO → SIMULATEUR 2D PYTHON
=========================================================
Gazebo était trop instable sur WSL2 (segfaults ThrusterPlugin, gz-transport
inaccessible, dépendances OpenGL). Cette stack utilise désormais le simulateur
2D Python pur (asket_ec_sim2d) à la place.

Lance en une commande :
  1. simulator_node  (asket_ec_sim2d) — physique 50 Hz + GPS simulé
  2. RViz2 avec config sim2d.rviz     — trajectoire verte + flèche de pose
  3. differential_drive_node           — Twist → commandes thrusters

Commande :
  ros2 launch asket_ec_gazebo beatnaut_full.launch.py

Pour envoyer des commandes manuelles :
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}, angular: {z: 0.2}}' --once
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lance la stack complète Sim2D + RViz2 + contrôleur."""

    pkg_sim2d = get_package_share_directory('asket_ec_sim2d')

    # use_sim_time=false : le simulateur Python publie sur l'horloge système.
    # Mettre à true uniquement si un nœud externe publie /clock.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Temps de simulation (false = horloge système pour sim2d)'
    )

    # =========================================================
    # COMPOSANT 1 : Simulateur 2D + RViz2
    # =========================================================
    # sim2d.launch.py démarre simulator_node et rviz2.
    sim2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim2d, 'launch', 'sim2d.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # =========================================================
    # COMPOSANT 2 : Contrôleur différentiel
    # =========================================================
    # Reçoit /cmd_vel → publie sur /asket_ec/thruster/*/cmd.
    # (Dans le contexte sim2d, ces topics ne sont pas consommés par Gazebo
    # mais permettent de valider la logique de commande.)
    control_node = Node(
        package='asket_ec_control',
        executable='differential_drive_node',
        name='differential_drive_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wheel_separation': 0.50,
            'max_thrust_rpm': 300.0,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        sim2d_launch,    # Sim2D + RViz2 (démarre immédiatement)
        control_node,    # Contrôleur différentiel
    ])
