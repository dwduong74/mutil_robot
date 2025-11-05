import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mutil_bringup_share_dir = get_package_share_directory('mutil_bringup')
    amcl_params_file = os.path.join(
        mutil_bringup_share_dir,
        'param',
        'amcl.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params_file]
        )
    ])
