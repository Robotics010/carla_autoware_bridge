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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    host_argument = DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server')
    port_argument = DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server')
    timeout_argument = DeclareLaunchArgument(
            name='timeout',
            default_value='5',
            description='Time to wait for a successful connection to the CARLA server')
    town_argument = DeclareLaunchArgument(
            name='town',
            default_value='Town01',
            description=('Either use an available CARLA town (eg. "Town01")'
                         'or an OpenDRIVE file (ending in .xodr)')
        )
    view_argument = DeclareLaunchArgument(
            name='view',
            default_value='false',
            description=('Is third person view window is needed,'
                         'it can be used to manual control as well')
        )

    carla_autoware_bridge = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_autoware_bridge')),
         '/carla_ros_bridge.launch.py']),
      launch_arguments={
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'timeout': LaunchConfiguration('timeout'),
                'town': LaunchConfiguration('town'),
            }.items())

    spawn_ego_vehicle = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_autoware_bridge')),
         '/carla_autoware_ego_vehicle.launch.py']))

    raw_vehicle_converter = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_autoware_bridge')),
         '/raw_vehicle_converter.launch.py']))

    manual_control_window = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('carla_manual_control')),
         '/carla_manual_control.launch.py']),
      condition=IfCondition(LaunchConfiguration('view')))

    ld = LaunchDescription([
        host_argument,
        port_argument,
        timeout_argument,
        town_argument,
        carla_autoware_bridge,
        spawn_ego_vehicle,
        raw_vehicle_converter,
        view_argument,
        manual_control_window,
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
