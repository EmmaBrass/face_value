from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pi_number = LaunchConfiguration('pi_number')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pi_number',
            default_value='1',
            description='Number of the Pi (1, 2, 3, etc.)'
        ),
        Node(
            package='ros_pi',
            executable='cam_node',
            name='cam_node', # TODO figure out how to give unique node names.
            output='screen',  # <- this ensures logs are printed
            parameters=[
                {'cam_id': LaunchConfiguration("pi_number")},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])