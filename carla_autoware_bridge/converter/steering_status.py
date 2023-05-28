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
from carla_msgs.msg import CarlaEgoVehicleSteering
import numpy as np


class SteeringStatusConverter(Converter):

    def __init__(self) -> None:
        super().__init__()

    def _convert(self):
        if not isinstance(self._inbox, CarlaEgoVehicleSteering):
            raise RuntimeError(f'Input must be {CarlaEgoVehicleSteering}!')

        output_steering_status = SteeringReport()
        output_steering_status.steering_tire_angle = -self._inbox.steering_tire_angle
        self._outbox = output_steering_status
