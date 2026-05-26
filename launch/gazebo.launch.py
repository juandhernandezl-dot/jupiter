#!/usr/bin/env python3
"""
gazebo.launch.py — SIMULACIÓN + ROBOT FÍSICO
════════════════════════════════════════════════════════════
Lanza todos los nodos del proyecto en un único comando:

  ros2 launch zumo_1 gazebo.launch.py

Componentes:
  1. micro_ros_agent  udp4 :8888   ← puente con el ESP32
  2. Gazebo Classic               ← simulador
  3. robot_state_publisher        ← TF + /robot_description
  4. joint_state_publisher        ← /joint_states
  5. spawn_entity                 ← inserta el robot en Gazebo (delay 3 s)
  6. rviz2                        ← visualización
  7. joy_node                     ← Xbox -> /joy
  8. teleop_twist_joy             ← /joy -> /cmd_vel

NOTA: el agente usa RMW_IMPLEMENTATION=rmw_fastrtps_cpp para
que el ESP32 y todos los nodos ROS compartan el mismo RMW
(Fast-DDS). Agrega esto a tu ~/.bashrc:
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
════════════════════════════════════════════════════════════
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'zumo_1'
    pkg_share = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Usar reloj de simulación (Gazebo)'
    )

    # ── URDF ──────────────────────────────────────────────────────────────────
    urdf_path = os.path.join(pkg_share, 'urdf', 'ROBOT_ZUMO_URDF.SLDASM.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_name_in_model = 'ROBOT_ZUMO_URDF.SLDASM'

    # ── 1. micro-ROS Agent ────────────────────────────────────────────────────
    # RMW_IMPLEMENTATION=rmw_fastrtps_cpp garantiza que el agente y los nodos
    # ROS usen el mismo middleware (Fast-DDS). Sin esto el ESP32 puede mostrar
    # 'session established' pero nunca recibir /cmd_vel (Subscription count: 0).
    microros_agent = ExecuteProcess(
        cmd=['bash', '-c',
             'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && '
             'source ~/microros_ws/install/local_setup.bash && '
             'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6'],
        output='screen',
        name='micro_ros_agent',
    )

    # ── 2. Gazebo Classic ─────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so',
        ],
        output='screen',
    )

    # ── 3. robot_state_publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }]
    )

    # ── 4. joint_state_publisher ──────────────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 5. spawn_entity (delay 3 s — espera que Gazebo cargue) ───────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-topic', '/robot_description',
                    '-entity', robot_name_in_model,
                    '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0',
                ],
            )
        ]
    )

    # ── 6. RViz2 ──────────────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 7 + 8. Teleop Xbox (joy_node + teleop_twist_joy) ─────────────────────
    # Se retrasa 3 s igual que spawn, para que /cmd_vel tenga al ESP32
    # emparejado antes de empezar a publicar.
    teleop_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('teleop_twist_joy'),
                        'launch', 'teleop-launch.py'
                    )
                ),
                launch_arguments={'joy_config': 'xbox'}.items()
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,

        # Primero el agente: el ESP32 puede conectarse mientras
        # Gazebo y los demás nodos terminan de arrancar.
        microros_agent,

        gazebo,
        robot_state_publisher,
        joint_state_publisher,

        # Con delay: necesitan que Gazebo y el agente estén listos.
        spawn_robot,    # 3 s
        teleop_launch,  # 3 s

        rviz2,
    ])