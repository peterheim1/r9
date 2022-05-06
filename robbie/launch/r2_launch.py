from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
         Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'scanner_link',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
             
        Node(
            package='r2_bringup',
            #namespace='robbie',
            executable='arduino',
            name='base_driver'
        ),
        
        Node(
            package='r2_bringup',
            #namespace='robbie',
            executable='arm_driver',
            name='arm_driver'
        ),
        
        
        Node(
            package='r2_bringup',
            #namespace='robbie',
            executable='voice_serv',
            name='voice_driver'
        )
    ])
