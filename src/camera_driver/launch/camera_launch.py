from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_driver',
            name='camera_driver',
            output='screen'
        )
    ])