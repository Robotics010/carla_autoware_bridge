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

from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from carla_autoware_bridge.bridge import AutowareBridge
from carla_msgs.msg import (
    CarlaEgoVehicleControl,
    CarlaEgoVehicleStatus,
    CarlaEgoVehicleSteering)
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped

from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.node import Node


class BridgeTester(Node):

    def __init__(self) -> None:
        super().__init__('bridge_tester')

        # Publishers
        self.odometry = None
        self._odometry_publisher = self.create_publisher(
            Odometry, '/carla_autoware_bridge/input/odometry', 1)

        self.vehicle_status = None
        self._vehicle_status_publisher = self.create_publisher(
            CarlaEgoVehicleStatus, '/carla_autoware_bridge/input/status', 1)

        self.vehicle_steering = None
        self._vehicle_steering_publisher = self.create_publisher(
            CarlaEgoVehicleSteering, '/carla_autoware_bridge/input/steering', 1)

        self.control_command = None
        self._control_command_publisher = self.create_publisher(
            ActuationCommandStamped, '/carla_autoware_bridge/input/actuation', 1)

        # Subscribers
        self.is_velocity_report_received = False
        self.velocity_report = None
        self._velocity_report_subscriber = self.create_subscription(
            VelocityReport, '/carla_autoware_bridge/output/velocity_status',
            self._velocity_report_callback, 1)
        self._velocity_report_subscriber

        self.is_steering_status_received = False
        self.steering_status = None
        self._steering_status_subscriber = self.create_subscription(
            SteeringReport, '/carla_autoware_bridge/output/steering_status',
            self._steering_status_callback, 1)
        self._steering_status_subscriber

        self.is_vehicle_control_command_received = False
        self.vehicle_control_command = None
        self._vehicle_control_command_subscriber = self.create_subscription(
            CarlaEgoVehicleControl, '/carla_autoware_bridge/output/control',
            self._vehicle_control_command_callback, 1)
        self._vehicle_control_command_subscriber
        
        self.is_actuation_status_received = False
        self.actuation_status = None
        self._actuation_status_subscriber = self.create_subscription(
            ActuationStatusStamped, '/carla_autoware_bridge/output/actuation_status',
            self._actuation_status_callback, 1)

    def _velocity_report_callback(self, velocity_report_msg):
        print(f'velocity report callback')
        self.velocity_report = velocity_report_msg
        self.is_velocity_report_received = True

    def _steering_status_callback(self, steering_status_msg):
        self.steering_status = steering_status_msg
        self.is_steering_status_received = True

    def _vehicle_control_command_callback(self, vehicle_control_command_msg):
        self.vehicle_control_command = vehicle_control_command_msg
        self.is_vehicle_control_command_received = True
    
    def _actuation_status_callback(self, actuation_status_msg):
        self.actuation_status = actuation_status_msg
        self.is_actuation_status_received = True

    def publish_odometry(self):
        self._odometry_publisher.publish(self.odometry)

    def publish_vehicle_status(self):
        self._vehicle_status_publisher.publish(self.vehicle_status)

    def publish_vehicle_steering(self):
        self._vehicle_steering_publisher.publish(self.vehicle_steering)

    def publish_control_command(self):
        self._control_command_publisher.publish(self.control_command)


def test_velocity_report():
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
    while not bridge_tester.is_velocity_report_received:
        rclpy.spin_once(bridge, timeout_sec=0.001)
        rclpy.spin_once(bridge_tester, timeout_sec=0.001)
        if time.time() - start_time > 0.1:
            raise RuntimeError('Exceeded the waiting timer for receiving odom msg!')

    assert pytest.approx(bridge_tester.velocity_report.heading_rate) == \
        pytest.approx(expected_velocity_report.heading_rate)
    assert pytest.approx(bridge_tester.velocity_report.longitudinal_velocity) == \
        pytest.approx(expected_velocity_report.longitudinal_velocity)
    assert pytest.approx(bridge_tester.velocity_report.lateral_velocity) == \
        pytest.approx(expected_velocity_report.lateral_velocity)
    rclpy.shutdown()


def test_steering_status():
    rclpy.init()
    bridge = AutowareBridge()

    fl_max_right_angle = 35.077
    fr_max_right_angle = 48.99
    average_max_right_angle = (fl_max_right_angle + fr_max_right_angle) / 2
    average_max_right_angle = np.radians(average_max_right_angle)

    input_vehicle_steering = CarlaEgoVehicleSteering()
    input_vehicle_steering.steering_tire_angle = average_max_right_angle

    expected_steering_tire_angle = -average_max_right_angle
    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = expected_steering_tire_angle

    bridge_tester = BridgeTester()
    bridge_tester.vehicle_steering = input_vehicle_steering
    bridge_tester.publish_vehicle_steering()

    start_time = time.time()
    while not bridge_tester.is_steering_status_received:
        rclpy.spin_once(bridge, timeout_sec=0.001)
        rclpy.spin_once(bridge_tester, timeout_sec=0.001)
        if time.time() - start_time > 1.0:
            raise RuntimeError('Exceeded the waiting timer for receiving steering_status msg!')

    assert pytest.approx(bridge_tester.steering_status.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)

    rclpy.shutdown()


def test_control_command():
    rclpy.init()
    bridge = AutowareBridge()

    input_control_command = ActuationCommandStamped()
    input_control_command.actuation.accel_cmd = 0.301

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.throttle = 0.301
    expected_control_command.brake = 0.0

    bridge_tester = BridgeTester()
    bridge_tester.control_command = input_control_command
    bridge_tester.publish_control_command()

    start_time = time.time()
    while not bridge_tester.is_vehicle_control_command_received:
        rclpy.spin_once(bridge, timeout_sec=0.001)
        rclpy.spin_once(bridge_tester, timeout_sec=0.001)
        if time.time() - start_time > 1.0:
            raise RuntimeError('Exceeded the waiting timer for '
                               'receiving vehicle_control_command msg!')

    assert pytest.approx(bridge_tester.vehicle_control_command.throttle) == \
        pytest.approx(expected_control_command.throttle)

    rclpy.shutdown()

def test_actuation_status():
    rclpy.init()
    bridge = AutowareBridge()

    input_vehicle_status = CarlaEgoVehicleStatus()
    input_vehicle_status.control.throttle = 0.4
    input_vehicle_status.control.brake = 0.0
    input_vehicle_status.control.steer = -0.1

    expected_actuation_status = ActuationStatusStamped()
    expected_actuation_status.status.accel_status = 0.4
    expected_actuation_status.status.brake_status = 0.0
    expected_actuation_status.status.steer_status = -0.1

    bridge_tester = BridgeTester()
    bridge_tester.vehicle_status = input_vehicle_status
    bridge_tester.publish_vehicle_status()

    start_time = time.time()
    while not bridge_tester.is_actuation_status_received:
        rclpy.spin_once(bridge, timeout_sec=0.001)
        rclpy.spin_once(bridge_tester, timeout_sec=0.001)
        if time.time() - start_time > 1.0:
            raise RuntimeError('Exceeded the waiting timer for receiving actuation_status msg!')

    assert pytest.approx(bridge_tester.actuation_status.status.accel_status) == \
        pytest.approx(expected_actuation_status.status.accel_status)
    assert pytest.approx(bridge_tester.actuation_status.status.brake_status) == \
        pytest.approx(expected_actuation_status.status.brake_status)
    assert pytest.approx(bridge_tester.actuation_status.status.steer_status) == \
        pytest.approx(expected_actuation_status.status.steer_status)

    rclpy.shutdown()
