import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Chemins vers nos nouveaux paquets
    pkg_description = get_package_share_directory('rover_description')
    pkg_control = get_package_share_directory('rover_control')  # <--- On pointe vers le nouveau paquet
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 1. Charger le robot (URDF)
    xacro_file = os.path.join(pkg_description, 'urdf', 'rover.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file, ' use_sim:=true'])}

    # 2. Lancer Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # 3. Faire apparaître le robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover'],
        output='screen'
    )

    # 4. Publier les états du robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. Charger les contrôleurs (Config lue automatiquement par Gazebo via ros2_control,
    # mais on doit lancer les spawners)
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner,
        diff_drive_spawner
    ])