import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # On va chercher ton fichier URDF
    pkg_description = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'rover.urdf.xacro')

    return LaunchDescription([
        # 1. Robot State Publisher (Lit l'URDF et le publie)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file, ' use_sim:=true'])
            }]
        ),
        
        # 2. Joint State Publisher GUI (La petite fenêtre pour bouger les roues)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # 3. RViz2 (L'écran de visualisation)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])