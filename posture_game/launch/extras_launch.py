from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        # Nodo de validación de posturas
        Node(
            package='posture_game',
            executable='checker_node',
            output='screen'
        ),

        # Nodo de voz TTS
        Node(
            package='posture_game',
            executable='speaker_node',
            output='log'
        ),

        Node(
            package='posture_game',
            executable='detect_emotions_node',
            output='log'
        ),

        # Nodo de la cámara (USB_CAM)
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
 
    ])
