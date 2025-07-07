from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=['/home/erickgonza/ros2_tesis/src/posture_game/config/v4l2_params.yaml']
        )
    ])
