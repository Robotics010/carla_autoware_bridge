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

import time

from autoware_auto_vehicle_msgs.msg import VelocityReport
from carla_autoware_bridge.bridge import AutowareBridge
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node


class BridgeTester(Node):

    def __init__(self) -> None:
        super().__init__('bridge_tester')

        self.odometry = None
        self.is_received = False
        self.velocity_report = None

        self._odometry_publisher = self.create_publisher(
            Odometry, '/carla/ego_vehicle/odometry', 1)
        self._velocity_report_subscriber = self.create_subscription(
            VelocityReport, '/vehicle/status/velocity_status',
            self._velocity_report_callback, 1)
        self._velocity_report_subscriber

    def _velocity_report_callback(self, velocity_report_msg):
        self.velocity_report = velocity_report_msg
        self.is_received = True

    def publish_odometry(self):
        self._odometry_publisher.publish(self.odometry)


def test_bridge_init():
    rclpy.init()
    bridge = AutowareBridge()

    input_odometry = Odometry()
    angular_velocity = input_odometry.twist.twist.angular
    angular_velocity.z = -0.317
    linear_velocity = input_odometry.twist.twist.linear
    linear_velocity.x = 5.55
    linear_velocity.y = 0.77

    expected_velocity_report = VelocityReport()
    expected_velocity_report.heading_rate = 0.317
    expected_velocity_report.longitudinal_velocity = 5.55
    expected_velocity_report.lateral_velocity = -0.77

    bridge_tester = BridgeTester()
    bridge_tester.odometry = input_odometry
    bridge_tester.publish_odometry()

    start_time = time.time()
    while not bridge_tester.is_received:
        rclpy.spin_once(bridge)
        rclpy.spin_once(bridge_tester)
        if time.time() - start_time > 0.1:
            raise RuntimeError('Exceeded the waiting timer for receiving odom msg!')
        time.sleep(0.001)

    assert pytest.approx(bridge_tester.velocity_report.heading_rate) == \
        pytest.approx(expected_velocity_report.heading_rate)
    assert pytest.approx(bridge_tester.velocity_report.longitudinal_velocity) == \
        pytest.approx(expected_velocity_report.longitudinal_velocity)
    assert pytest.approx(bridge_tester.velocity_report.lateral_velocity) == \
        pytest.approx(expected_velocity_report.lateral_velocity)
    rclpy.shutdown()
