from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='towerlight_maker',
            executable='tower_udp_client',
            name='tower_udp_client',
            output='screen',
            parameters=[{
                'target_ip': '10.1.100.200',
                'target_port': 9000,
                'qos_reliable': False,
            }],
        ),
    ])
