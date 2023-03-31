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


from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from carla_autoware_bridge.converter.fake import FakeConverter
from carla_autoware_bridge.converter.steering_status import SteeringStatusConverter
from carla_autoware_bridge.converter.velocity_report import VelocityReportConverter
from carla_msgs.msg import CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry
import numpy as np

import pytest


def test_fake_convert_without_setting_inbox():
    fake_converter = FakeConverter()
    with pytest.raises(RuntimeError):
        fake_converter.convert()


def test_fake_outbox_without_calling_convert():
    input_value = 10
    fake_converter = FakeConverter()
    fake_converter.inbox = input_value
    with pytest.raises(RuntimeError):
        fake_converter.outbox


def test_fake_inbox():
    input_value = 10
    fake_converter = FakeConverter()
    fake_converter.inbox = input_value
    assert fake_converter.inbox == input_value


def test_fake_outbox_read_only():
    output_value = 10
    fake_converter = FakeConverter()
    with pytest.raises(AttributeError):
        fake_converter.outbox = output_value


def test_fake_convert():
    input_value = 10
    fake_converter = FakeConverter()
    fake_converter.inbox = input_value
    fake_converter.convert()
    assert fake_converter.outbox == input_value


def test_velocity_report_convert():
    input_odometry = Odometry()
    angular_velocity = input_odometry.twist.twist.angular
    angular_velocity.z = -0.317
    linear_velocity = input_odometry.twist.twist.linear
    linear_velocity.x = 5.55
    linear_velocity.y = 0.77

    output_velocity_report = VelocityReport()
    output_velocity_report.heading_rate = 0.317
    output_velocity_report.longitudinal_velocity = 5.55
    output_velocity_report.lateral_velocity = -0.77

    vel_rep_converter = VelocityReportConverter()
    vel_rep_converter.inbox = input_odometry
    vel_rep_converter.convert()
    assert vel_rep_converter.outbox.longitudinal_velocity == \
        output_velocity_report.longitudinal_velocity
    assert vel_rep_converter.outbox.lateral_velocity == \
        output_velocity_report.lateral_velocity
    assert vel_rep_converter.outbox.heading_rate == output_velocity_report.heading_rate


def test_velocity_report_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    vel_rep_converter = VelocityReportConverter()
    vel_rep_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        vel_rep_converter.convert()


def test_left_steering_status_convert():
    input_vehicle_status = CarlaEgoVehicleStatus()
    input_vehicle_status.control.steer = -1.0

    fl_max_left_angle = -48.99
    fr_max_left_angle = -35.077
    average_max_left_angle = (fl_max_left_angle + fr_max_left_angle) / 2

    expected_steering_tire_angle = -np.radians(average_max_left_angle)
    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = expected_steering_tire_angle

    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_vehicle_status
    steering_status_converter.convert()

    assert pytest.approx(steering_status_converter.outbox.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)


def test_right_steering_status_convert():
    input_vehicle_status = CarlaEgoVehicleStatus()
    input_vehicle_status.control.steer = 1.0

    fl_max_right_angle = 35.077
    fr_max_right_angle = 48.99
    average_max_right_angle = (fl_max_right_angle + fr_max_right_angle) / 2

    expected_steering_tire_angle = -np.radians(average_max_right_angle)
    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = expected_steering_tire_angle

    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_vehicle_status
    steering_status_converter.convert()

    assert pytest.approx(steering_status_converter.outbox.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)


def test_center_steering_status_convert():
    input_vehicle_status = CarlaEgoVehicleStatus()
    input_vehicle_status.control.steer = 0.0

    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = 0.0

    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_vehicle_status
    steering_status_converter.convert()

    assert pytest.approx(steering_status_converter.outbox.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)


def test_steering_status_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        steering_status_converter.convert()
