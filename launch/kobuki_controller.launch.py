import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # zed_wrapper_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('zed_wrapper'), 'launch'),
    #         '/zed2.launch.py'])
    #     )

    return launch.LaunchDescription([
        # zed_wrapper_launch,
        launch_ros.actions.Node(
            package='kobuki_controller',
            executable='kobuki_controller',
            name='kobuki_controller'),
        launch_ros.actions.Node(
            package='ugrdv_kobuki_ros',
            executable='ugrdv_kobuki_ros',
            name='ugrdv_kobuki_ros'),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy',
        )
    ])
