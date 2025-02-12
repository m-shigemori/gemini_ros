from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="gemini_ros",
            executable="gemini_vlm",
            name="gemini_vlm_node",
            output="screen",
            emulate_tty=True
        )
    ])