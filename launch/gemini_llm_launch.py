from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gemini_ros",
            executable="gemini_llm",
            name="gemini_llm_node",
            output="screen",
            emulate_tty=True
        )
    ])