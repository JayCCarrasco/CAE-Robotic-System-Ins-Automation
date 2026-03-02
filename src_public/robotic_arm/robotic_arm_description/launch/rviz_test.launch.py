from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare 

def generate_launch_description():
    # Ruta al xacro
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("robotic_arm_description"), "urdf", "kr6r900sixx.xacro"]
    )

    # Comando para generar URDF desde xacro
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('robotic_arm_description'), 'rviz', 'robot.rviz'])]
        )
    ])