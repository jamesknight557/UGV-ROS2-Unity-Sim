from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('unity_slam_bringup')
    slam_params = os.path.join(bringup_share, 'config', 'slam_params.yaml')

    return LaunchDescription([
        Node(
            package='unity_slam_bringup',
            executable='odom_to_tf_broadcaster',
            name='odom_to_tf_broadcaster',
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),
    ])

