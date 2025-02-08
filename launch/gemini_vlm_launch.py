from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gemini_vlm_node = Node(
        package='gemini_ros',
        executable='gemini_vlm',
        name='gemini_vlm_node',
        output='screen',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ])
    )

    return LaunchDescription([
        gemini_vlm_node,
        # realsense_launch
    ])
