import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    package_share = FindPackageShare('rovio').find('rovio')
    config_file = os.path.join(package_share, 'cfg', 'rovio.info')
    filter_config_arg = DeclareLaunchArgument('filter_config',
                                              default_value=config_file)
    cam0_config = os.path.join(package_share,'cfg', 'euroc_cam0.yaml')
    cam0_config_arg = DeclareLaunchArgument('cam0_config',default_value=cam0_config)
    cam1_config = os.path.join(package_share,'cfg', 'euroc_cam1.yaml')
    cam1_config_arg = DeclareLaunchArgument('cam1_config',default_value=cam1_config)
    print("cam1 config: ", cam1_config)
    print("cam0 config: ", cam0_config)
    print("filter config: ", config_file)
    rovio_node = Node(
        package='rovio',
        executable='rovio_node',
        name='rovio',
        output='screen',
        parameters=[
            {
                'filter_config': LaunchConfiguration('filter_config'),
                'cam0_config': LaunchConfiguration('cam0_config'),
                'cam1_config': LaunchConfiguration('cam1_config'),
            }
        ]
    )
    return LaunchDescription([
        filter_config_arg,
        cam0_config_arg,
        cam1_config_arg,
        rovio_node
    ])
