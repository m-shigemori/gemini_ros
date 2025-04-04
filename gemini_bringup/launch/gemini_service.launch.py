from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gemini_ros',
            executable='gemini_service_server',
            name='gemini_service_server'
        ),
        Node(
            package='gemini_ros',
            executable='gemini_service_client',
            name='gemini_service_client',
            arguments=["Hello, launch!"]
        ),
    ])