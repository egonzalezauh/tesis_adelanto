from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        #Nodo de escucha de cámara 
        #Node(
            #package='posture_game',
            #executable='camera_listener',
            #output='log'
        #),

        # Nodo de validación de posturas
        Node(
            package='posture_game',
            executable='checker_node',
            output='screen'
        ),

        #Nodo auxiliar (dummy para debug o test)
        #Node(
            #package='posture_game',
            #executable='dummy_subscriber',
            #output='log'
        #),

        # Nodo de voz TTS
        Node(
            package='posture_game',
            executable='speaker_node',
            output='log'
        ),

        # Nodo de la cámara (USB_CAM)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=['/home/erickgonza/ros2_tesis/src/posture_game/config/usb_cam_params.yaml'],
            output='screen'
        ),

        #Node(
            #package='v4l2_camera',
            #executable='v4l2_camera_node',
            #name='v4l2_camera',
            #output='log',
            #parameters=['/home/erickgonza/ros2_tesis/src/posture_game/config/v4l2_params.yaml']
        #)

 
    ])
