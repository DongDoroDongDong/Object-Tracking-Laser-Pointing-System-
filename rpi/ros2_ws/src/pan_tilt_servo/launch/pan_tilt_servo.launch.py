from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pan_tilt_servo',
            executable='pan_tilt_servo_node',
            name='pan_tilt_servo_node',
            output='screen',
            parameters=[{
                'pan_gpio': 13,
                'tilt_gpio': 19,
                'min_us': 500,
                'max_us': 2500,
                'pan_deg_min': -90.0,
                'pan_deg_max': 90.0,
                'tilt_deg_min': -45.0,
                'tilt_deg_max': 45.0,
                'alpha': 0.25,
                'apply_rate_hz': 50
            }]
        )
    ])
