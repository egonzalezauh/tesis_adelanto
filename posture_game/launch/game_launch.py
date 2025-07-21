from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='posture_game',
            executable='game_manage',
            output='screen'
        )
    ])
