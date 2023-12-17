from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='r2_bringup',
            #namespace='robbie',
            executable='arduino',
            name='arduino'
        )
        
    ])
