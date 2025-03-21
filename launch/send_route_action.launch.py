# Import the necessary launch libraries
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='luggage_av_msgs',
            executable='send_route_action_client',
            output='screen'
        ),
    ])