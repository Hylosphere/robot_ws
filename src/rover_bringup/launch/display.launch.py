import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    pkg_description = get_package_share_directory('rover_description')
    
    # Chemins des fichiers
    xacro_file = os.path.join(pkg_description, 'urdf', 'rover.urdf.xacro')
    rviz_config_file = os.path.join(pkg_bringup, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        # 1. Publie l'état du robot (C'est lui qui lit le URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                # ⚠️ LA CORRECTION EST ICI : use_sim:=false
                'robot_description': Command(['xacro ', xacro_file, ' use_sim:=false'])
            }]
        ),
        
        # 2. Permet de bouger les roues manuellement
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # 3. Lance RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])