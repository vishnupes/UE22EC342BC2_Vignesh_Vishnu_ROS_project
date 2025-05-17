from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to files
    pkg_path = get_package_share_directory('mars_rover')
    urdf_path = os.path.join(pkg_path, 'urdf', 'mars_rover.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'mars.world')

    return LaunchDescription([
        # Launch Gazebo with the custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),

        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        
        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-entity', 'mars_rover', '-topic', 'robot_description'],
            output='screen'
        ),
    ])