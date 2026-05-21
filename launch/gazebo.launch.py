import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'zumo_1'
    pkg_share     = get_package_share_directory(package_name)

    use_sim_time  = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Usar reloj de simulación (Gazebo)'
    )

    # ── URDF ─────────────────────────────────────────────────
    urdf_path = os.path.join(pkg_share, 'urdf', 'ROBOT_ZUMO_URDF.SLDASM.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_name_in_model = 'ROBOT_ZUMO_URDF.SLDASM'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name_in_model,
            '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0'
        ]
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Teleop Xbox ───────────────────────────────────────────
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_twist_joy'),
                'launch', 'teleop-launch.py'
            )
        ),
        launch_arguments={'joy_config': 'xbox'}.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        rviz2,
        teleop_launch,
    ])
