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
    shaders_vs = os.path.join(package_share, 'shaders', 'shaders.vs')
    shaders_vsFileName_arg = DeclareLaunchArgument('shaders_mVSFileName',
                                                    default_value=shaders_vs)
    shaders_fs = os.path.join(package_share, 'shaders', 'shaders.fs')
    shaders_fsFileName_arg = DeclareLaunchArgument('shaders_mFSFileName',
                                                   default_value= shaders_fs)
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
                'camera0_config': LaunchConfiguration('cam0_config'),
                'camera1_config': LaunchConfiguration('cam1_config'),
                'shaders_mVSFileName': LaunchConfiguration('shaders_mVSFileName'),
                'shaders_mFSFileName': LaunchConfiguration('shaders_mFSFileName')
            }
        ]
    )
    return LaunchDescription([
        filter_config_arg,
        cam0_config_arg,
        cam1_config_arg,
        shaders_vsFileName_arg,
        shaders_fsFileName_arg,
        rovio_node
    ])
