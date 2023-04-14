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

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from carla_autoware_bridge.converter.control_command import ControlCommandConverter
from carla_autoware_bridge.converter.lidar_ex import LidarExtendedConverter
from carla_autoware_bridge.converter.steering_status import SteeringStatusConverter
from carla_autoware_bridge.converter.velocity_report import VelocityReportConverter
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class AutowareBridge(Node):

    def __init__(self) -> None:
        super().__init__('carla_autoware_bridge')
        self._velocity_report_converter = VelocityReportConverter()
        self._steering_status_converter = SteeringStatusConverter()
        self._control_command_converter = ControlCommandConverter()
        self._lidar_ex_converter = LidarExtendedConverter()

        self._odometry_subscriber = self.create_subscription(
            Odometry, '/carla/ego_vehicle/odometry',
            self._odometry_callback, 1)

        self._vehicle_status_subscriber = self.create_subscription(
            CarlaEgoVehicleStatus, '/carla/ego_vehicle/vehicle_status',
            self._vehicle_status_callback, 1)

        self._ackermann_control_command_subscriber = self.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd',
            self._ackermann_control_command_callback, 1)

        self._lidar_subscriber = self.create_subscription(
            PointCloud2, '/carla/ego_vehicle/lidar',
            self._lidar_callback, 1)

        self._velocity_report_publisher = self.create_publisher(
            VelocityReport, '/carla/ego_vehicle/velocity_status', 1)

        self._steering_status_publisher = self.create_publisher(
            SteeringReport, '/carla/ego_vehicle/steering_status', 1)

        self._vehicle_control_command_publisher = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 1)

        self._lidar_ex_publisher = self.create_publisher(
            PointCloud2, '/carla/ego_vehicle/lidar_ex', 1)

    def _odometry_callback(self, odometry_msg):
        self._velocity_report_converter.inbox = odometry_msg
        self._velocity_report_converter.convert()

        velocity_report_msg = self._velocity_report_converter.outbox

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = self.get_clock().now().to_msg()
        velocity_report_msg.header = header
        self._velocity_report_publisher.publish(velocity_report_msg)

    def _vehicle_status_callback(self, vehicle_status_msg):
        self._steering_status_converter.inbox = vehicle_status_msg
        self._steering_status_converter.convert()

        steering_status_msg = self._steering_status_converter.outbox
        steering_status_msg.stamp = self.get_clock().now().to_msg()
        self._steering_status_publisher.publish(steering_status_msg)

    def _ackermann_control_command_callback(self, ackermann_control_command_msg):
        self._control_command_converter.inbox = ackermann_control_command_msg
        self._control_command_converter.convert()

        vehicle_control_command_msg = self._control_command_converter.outbox
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        vehicle_control_command_msg.header = header
        self._vehicle_control_command_publisher.publish(vehicle_control_command_msg)

    def _lidar_callback(self, lidar_msg):
        self._lidar_ex_converter.inbox = lidar_msg
        self._lidar_ex_converter.convert()

        lidar_ex_msg = self._lidar_ex_converter.outbox
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = lidar_msg.header.frame_id
        lidar_ex_msg.header = header
        self._lidar_ex_publisher.publish(lidar_ex_msg)
