from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    #teleop_node = Node(
     #   package='teleop_twist_keyboard',
      #  executable='teleop_twist_keyboard',
       # name='teleop_keyboard',
      #  output='screen',
     #   emulate_tty=True,
      #  remappings=[('/cmd_vel', '/cmd_vel_teleop')]
    #)

    control_mode_manager_node = Node(
        package='mobile_robot_manager',
        executable='control_mode_manager',
        name='control_mode_manager',
        output='screen'
    )

    safety_monitor_node = Node(
        package = 'mobile_robot_monitor',
        executable = 'safety_monitor',
        name = 'safety_monitor',
        output = 'screen',
        parameters=['config/safety_monitor.yaml']
    )

    return LaunchDescription([
        #teleop_node,
        control_mode_manager_node,
        safety_monitor_node
    ])