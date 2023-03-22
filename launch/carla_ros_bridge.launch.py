import os

import launch
from launch.actions import GroupAction
from launch.actions import SetRemap
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # carla_ros_bridge_loader = launch.LaunchDescription([
    #     IncludeLaunchDescription(
    #         launch.launch_description_sources.PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory(
    #                 'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
    #         )
    #     )
    # ])
    
    # ToDo(Robotics010): Change to launch with SetRemap when switch to Foxy or later
    # group = GroupAction(actions=[
    #     SetRemap(src='/carla/ego_vehicle/rgb_view/camera_info',dst='/sensing/camera/traffic_light/camera_info'),
    #     SetRemap(src='/carla/ego_vehicle/rgb_view/image',dst='/sensing/camera/traffic_light/image_raw'),
    #     SetRemap(src='/carla/ego_vehicle/gnss',dst='/sensing/gnss/ublox/nav_sat_fix'),
    #     SetRemap(src='/carla/ego_vehicle/imu',dst='/sensing/imu/tamagawa/imu_raw'),
    #     SetRemap(src='/carla/ego_vehicle/lidar',dst='/sensing/lidar/top/pointcloud_raw_ex'),
    #     SetRemap(src='/carla/ego_vehicle/velocity_status',dst='/vehicle/status/velocity_status'),
    #     carla_ros_bridge_loader,
    # ])
    
    return group


if __name__ == '__main__':
    generate_launch_description()
