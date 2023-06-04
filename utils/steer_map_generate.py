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

from autoware_auto_vehicle_msgs.msg import SteeringReport
from carla_msgs.msg import CarlaEgoVehicleControl
import numpy as np
import rclpy
from rclpy.node import Node


class SteeringMapCollector(Node):

    def __init__(self):
        super().__init__('steer_map_collector')

        self._vehicle_control_publisher = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 1)

        self._steer_range = np.linspace(1.0, -1.0, 1401)
        self._prev_tire_angle = 0.0
        self._tire_angles = []
        self._convergence_count = 0
        self._convergence_count_max = 3

        self._steering_status_subscriber = self.create_subscription(
            SteeringReport, '/vehicle/status/steering_status',
            self._steering_status_callback, 1)

        self._control_index = 0
        self._steer_cmd = self._steer_range[self._control_index]
        cmd_pub_period = 1.0 / 10.00  # seconds
        self._pub_timer = self.create_timer(cmd_pub_period, self._on_publish)
        self._map_saved = False

    def _on_publish(self):
        msg_control_cmd = CarlaEgoVehicleControl()
        msg_control_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_control_cmd.steer = self._steer_cmd
        self._vehicle_control_publisher.publish(msg_control_cmd)

    def _steering_status_callback(self, steering_status_msg):
        tire_angle = steering_status_msg.steering_tire_angle
        if self._prev_tire_angle == tire_angle:
            self._convergence_count += 1
        if self._convergence_count >= self._convergence_count_max:
            self._convergence_count = 0
            self._tire_angles.append(tire_angle)
            if self._control_index + 1 != len(self._steer_range):
                self._control_index += 1
                self._steer_cmd = self._steer_range[self._control_index]
            else:
                self._steer_cmd = 0.0
                if not self._map_saved:
                    self._map_saved = True
                    with open('steer_map.csv', 'w') as csvfile:
                        csv_writer = csv.writer(csvfile, delimiter=',')
                        csv_writer.writerow(self._tire_angles)
                        csv_writer.writerow(self._steer_range)

        self._prev_tire_angle = tire_angle


def main():
    rclpy.init()
    steering_collector = SteeringMapCollector()

    rclpy.spin(steering_collector)

    steering_collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
