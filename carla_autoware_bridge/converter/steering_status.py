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

from autoware_auto_vehicle_msgs.msg import SteeringReport
from carla_autoware_bridge.converter.converter import Converter
from carla_msgs.msg import CarlaEgoVehicleStatus
import numpy as np


class SteeringStatusConverter(Converter):

    def __init__(self) -> None:
        super().__init__()
        self._steer_to_angle_polynomial = self._calculate_steer_to_angle_polynomial()

    def _calculate_steer_to_angle_polynomial(self):
        max_left_steer = -1
        max_right_steer = 1

        fl_max_left_angle = -48.99
        fr_max_left_angle = -35.077
        average_max_left_angle = -(fl_max_left_angle + fr_max_left_angle) / 2
        average_max_left_angle = np.radians(average_max_left_angle)
        average_max_right_angle = -average_max_left_angle

        x = [max_left_steer, max_right_steer]
        y = [average_max_left_angle, average_max_right_angle]
        polynomial_coefficients = np.polyfit(x, y, 1)
        steer_to_angle_polynomial = np.poly1d(polynomial_coefficients)
        return steer_to_angle_polynomial

    def _convert(self):
        if not isinstance(self._inbox, CarlaEgoVehicleStatus):
            raise RuntimeError(f'Input must be {CarlaEgoVehicleStatus}!')

        output_steering_status = SteeringReport()
        steer = self._inbox.control.steer
        angle = self._steer_to_angle_polynomial(steer)
        output_steering_status.steering_tire_angle = angle
        self._outbox = output_steering_status
