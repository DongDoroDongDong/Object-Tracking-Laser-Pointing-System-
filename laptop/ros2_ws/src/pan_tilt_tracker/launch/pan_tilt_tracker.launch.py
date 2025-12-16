from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('pan_tilt_tracker')
    params_file = os.path.join(share, 'config', 'pan_tilt_tracker.yaml')

    return LaunchDescription([
        Node(
            package='pan_tilt_tracker',
            executable='pan_tilt_tracker_node',
            name='pan_tilt_tracker',
            output='screen',
            parameters=[params_file],
        )
    ])
