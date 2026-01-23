from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mobile_robot_teleop'),
        'config',
        'teleop_joy.yaml'
    )

    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_joystick_node',
            parameters=[config_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel_manual')
            ]
        )
    ])

