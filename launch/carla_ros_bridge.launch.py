#   MIT License
#
#   Copyright (c) 2023 Robotics010
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='2',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description=('When enabled, the ROS bridge will take a backseat'
                         'and another client must tick the world (only in synchronous mode)')
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description=('Enable/disable synchronous mode.'
                         'If enabled, the ROS bridge waits'
                         'until the expected data is received for all sensors')
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description=('When enabled, pauses the tick until a vehicle control'
                         'is completed (only in synchronous mode)')
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01',
            description=('Either use an available CARLA town (eg. "Town01")'
                         'or an OpenDRIVE file (ending in .xodr)')
        ),
        launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True',
            description=('Enable/disable the registration of all sensors.'
                         'If disabled, only sensors spawned by the bridge are registered')
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=['hero', 'ego_vehicle', 'hero0', 'hero1', 'hero2',
                           'hero3', 'hero4', 'hero5', 'hero6', 'hero7', 'hero8', 'hero9'],
            description='Role names to identify ego vehicles. '
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_steer_map',
            default_value=get_package_share_directory(
                'carla_autoware_bridge') + '/data/carla_tesla_model3/steer_map.csv'
        ),
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'passive': launch.substitutions.LaunchConfiguration('passive')
                },
                {
                    'synchronous_mode': launch.substitutions.LaunchConfiguration(
                        'synchronous_mode')
                },
                {
                    'synchronous_mode_wait_for_vehicle_control_command':
                        launch.substitutions.LaunchConfiguration(
                            'synchronous_mode_wait_for_vehicle_control_command')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration(
                        'fixed_delta_seconds')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'register_all_sensors': launch.substitutions.LaunchConfiguration(
                        'register_all_sensors')
                },
                {
                    'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration(
                        'ego_vehicle_role_name')
                }
            ],
            remappings=[
                ('/carla/ego_vehicle/rgb_front/camera_info',
                 '/sensing/camera/traffic_light/camera_info'),
                ('/carla/ego_vehicle/rgb_front/image', '/sensing/camera/traffic_light/image_raw'),
                ('/carla/ego_vehicle/gnss', '/sensing/gnss/ublox/nav_sat_fix'),
                ('/carla/ego_vehicle/imu', '/sensing/imu/tamagawa/imu_raw'),
                ('/carla/ego_vehicle/lidar', '/sensing/lidar/top/pointcloud_raw'),
            ],
        ),
        launch_ros.actions.Node(
            package='carla_autoware_bridge',
            executable='carla_autoware_bridge',
            name='carla_autoware_bridge',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'csv_path_steer_map': launch.substitutions.LaunchConfiguration(
                        'csv_path_steer_map')
                }
            ],
            remappings=[
                ('~/input/odometry', '/carla/ego_vehicle/odometry'),
                ('~/input/status', '/carla/ego_vehicle/vehicle_status'),
                ('~/input/steering', '/carla/ego_vehicle/vehicle_steering'),
                ('~/input/actuation', '/control/command/actuation_cmd'),
                ('~/input/lidar', '/sensing/lidar/top/pointcloud_raw'),
                ('~/output/velocity_status', '/vehicle/status/velocity_status'),
                ('~/output/steering_status', '/vehicle/status/steering_status'),
                ('~/output/actuation_status', '/vehicle/status/actuation_status'),
                ('~/output/control', '/carla/ego_vehicle/vehicle_control_cmd'),
                ('~/output/lidar_ex', '/sensing/lidar/top/pointcloud_raw_ex'),
            ],
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
