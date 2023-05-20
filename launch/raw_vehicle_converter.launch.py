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

import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='converter_param_path',
            default_value=get_package_share_directory(
                'raw_vehicle_cmd_converter') + '/config/converter.param.yaml'
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_accel_map',
            default_value=get_package_share_directory(
                'carla_autoware_bridge') + '/data/carla_tesla_model3/accel_map.csv'
        ),
        launch.actions.DeclareLaunchArgument(
            name='csv_path_brake_map',
            default_value=get_package_share_directory(
                'carla_autoware_bridge') + '/data/carla_tesla_model3/brake_map.csv'
        ),
        launch_ros.actions.Node(
            package='raw_vehicle_cmd_converter',
            executable='raw_vehicle_cmd_converter_node',
            name='raw_vehicle_cmd_converter',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'csv_path_accel_map': launch.substitutions.LaunchConfiguration('csv_path_accel_map')
                },
                {
                    'convert_accel_cmd': True
                },
                {
                    'csv_path_brake_map': launch.substitutions.LaunchConfiguration('csv_path_brake_map')
                },
                {
                    'convert_brake_cmd': True
                },
                {
                    'convert_steer_cmd': False
                },
                {
                    'max_throttle': 0.8
                },
                {
                    'max_brake': 0.4
                },
                {
                    'max_steer': 1.0
                },
                {
                    'min_steer': -1.0
                },
                launch.substitutions.LaunchConfiguration('converter_param_path'),
            ],
            remappings=[
                ('~/input/control_cmd', '/control/command/control_cmd'),
                ('~/input/odometry', '/localization/kinematic_state'),
                ('~/input/steering', '/vehicle/status/steering_status'),
                ('~/output/actuation_cmd', '/control/command/actuation_cmd'),
            ],
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
