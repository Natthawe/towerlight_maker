from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    client = Node(
        package='towerlight_maker',
        executable='tower_udp_client',
        name='tower_udp_client',
        output='screen',
        parameters=[{
            'target_ip': '10.1.100.200',
            'target_port': 9000,
            'qos_reliable': False,
        }],
    )
    rules = Node(
        package='towerlight_maker',
        executable='tower_rules_bridge',
        name='tower_rules_bridge',
        output='screen',
        parameters=[{
            'bumper_topic': '/bumper_active',
            'emergency_topic': '/emergency',
            'distance_topic': '/closest_object_distance',
            'distance_msg_type': 'float32', 
            'distance_thresh': 0.5,
            'warn_track': 2,
            'emergency_track': 1,
            'warn_blink_ms': 450,
            'emer_blink_ms': 240,
            'retrigger_cooldown_sec': 3.0,
            'safe_light_refresh_sec': 5.0,
            'warn_light_refresh_sec': 0.0,
            'emer_light_refresh_sec': 0.0,
            'emergency_true_is_emergency': True,
            'bumper_true_is_ok': True,            
            'use_controller_status': True,
            'controller_status_topic': '/gmr_controllers/status',                 
        }],
    )
    return LaunchDescription([client, rules])
