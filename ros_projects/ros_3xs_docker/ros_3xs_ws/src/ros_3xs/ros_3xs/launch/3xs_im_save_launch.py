from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# This launch file will only launch a test node for saving images received from the RPis.

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_3xs_docker',
            executable='screen1_node',
            name='screen1_node',
            parameters=[
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])