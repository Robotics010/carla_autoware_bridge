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

import csv

from carla_autoware_bridge.converter.converter import Converter
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
from tier4_vehicle_msgs.msg import ActuationCommandStamped


class ControlCommandConverter(Converter):

    def __init__(self, steer_map_path='') -> None:
        super().__init__()

        with open(steer_map_path, newline='') as csvfile:
            csv_reader = csv.reader(csvfile)
            self._tire_angle = np.float32(next(csv_reader))
            self._steer_cmd = np.float32(next(csv_reader))

    def _convert_from_tire_to_steer(self, tire_angle) -> float:
        nearest_idx = (np.abs(self._tire_angle - tire_angle)).argmin()
        return float(self._steer_cmd[nearest_idx])

    def _convert(self):
        if not isinstance(self._inbox, ActuationCommandStamped):
            raise RuntimeError(f'Input must be {ActuationCommandStamped}!')

        steer = self._convert_from_tire_to_steer(self._inbox.actuation.steer_cmd)

        output_control_command = CarlaEgoVehicleControl()
        output_control_command.throttle = self._inbox.actuation.accel_cmd
        output_control_command.brake = self._inbox.actuation.brake_cmd
        output_control_command.manual_gear_shift = False
        output_control_command.steer = steer
        self._outbox = output_control_command
