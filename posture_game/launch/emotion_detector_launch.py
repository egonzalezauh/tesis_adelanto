from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[os.path.join(
                get_package_share_directory('posture_game'),
                'config',
                'usb_cam_params.yaml')],
            output='log'
        ),
        Node(
            package='posture_game',
            executable='detect_emotions_node',
            name='emotion_detector',
            output='screen'
        ),



    ])

