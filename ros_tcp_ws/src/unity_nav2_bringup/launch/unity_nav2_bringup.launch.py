from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')
    unity_nav2_share = get_package_share_directory('unity_nav2_bringup')

    nav2_params = os.path.join(unity_nav2_share, 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
