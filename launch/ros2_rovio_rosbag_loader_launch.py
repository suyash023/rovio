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
    bag_file_path = "/home/suby/Desktop/Robotics_projects/open_source/rovio_ws/datasets/machine_hall/MH_01_easy/MH_01_easy_ros2/MH_01_easy_ros2.db3"
    bag_file_arg = DeclareLaunchArgument('rosbag_filename', default_value=bag_file_path)
    imu_topic_arg = DeclareLaunchArgument('imu_topic_name', default_value='/imu0')
    cam0_topic_arg = DeclareLaunchArgument('cam0_topic_name', default_value='/cam0/image_raw')
    cam1_topic_arg = DeclareLaunchArgument('cam1_topic_name', default_value="/cam1/image_raw")
    gt_topic_arg = DeclareLaunchArgument('gt_topic_name', default_value="/leica/position")
    resize_image_arg = DeclareLaunchArgument('resize_image', default_value=False)
    resize_image_width_arg = DeclareLaunchArgument('resize_image_width', default_value= 320)
    resize_image_height_arg = DeclareLaunchArgument('resize_image_height', default_value=240)

    print("cam1 config: ", cam1_config)
    print("cam0 config: ", cam0_config)
    print("filter config: ", config_file)
    print("basg file path: ", bag_file_path)
    rovio_node = Node(
        package='rovio',
        executable='rovio_rosbag_loader',
        name='rovio',
        output='screen',
        parameters=[
            {
                'filter_config': LaunchConfiguration('filter_config'),
                'camera0_config': LaunchConfiguration('cam0_config'),
                'camera1_config': LaunchConfiguration('cam1_config'),
                'rosbag_filename' : LaunchConfiguration('rosbag_filename'),
                'cam0_topic_name' : LaunchConfiguration('cam0_topic_name'),
                'cam1_topic_name' : LaunchConfiguration('cam1_topic_name'),
                'imu_topic_name' : LaunchConfiguration('imu_topic_name'),
                'gt_topic_name' : LaunchConfiguration('gt_topic_name'),
                'resize_image' : LaunchConfiguration('resize_image'),
                'resize_image_width': LaunchConfiguration('resize_image_width'),
                'resize_image_height': LaunchConfiguration('resize_image_height')
            }
        ]
    )
    return LaunchDescription([
        filter_config_arg,
        cam0_config_arg,
        cam1_config_arg,
        bag_file_arg,
        cam0_topic_arg,
        cam1_topic_arg,
        imu_topic_arg,
        gt_topic_arg,
        resize_image_arg,
        resize_image_width_arg,
        resize_image_height_arg,
        rovio_node
    ])
