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

import threading

from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class CarlaService(Node):

    def __init__(self) -> None:
        super().__init__('carla_service')

        self._vehicle_status_msg = CarlaEgoVehicleStatus()

        self._vehicle_status_subscriber = self.create_subscription(
            CarlaEgoVehicleStatus, '~/input/vehicle_status',
            self._vehicle_status_callback, 1)

        self._vehicle_control_command_publisher = self.create_publisher(
            CarlaEgoVehicleControl, '~/output/control', 1)

    def _vehicle_status_callback(self, vehicle_status_msg):
        self._vehicle_status_msg = vehicle_status_msg

    def prepare_vehicle_control(self):
        """
        Prepare vehicle interface for auto mode.

        After spawning a car we have to switch from automatic transmission to manual transmission,
        put the car in the first gear and then switch back to automatic transmission,
        otherwise there is a delay when applying high throttle for the first time or
        if a low throttle is applied for the first time the vehicle will not move at all
        """
        output_control_command = CarlaEgoVehicleControl()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        output_control_command.header = header
        output_control_command.manual_gear_shift = True
        output_control_command.gear = 1
        self._set_control_and_wait_until_applied(output_control_command)

        header.stamp = self.get_clock().now().to_msg()
        output_control_command.header = header
        output_control_command.manual_gear_shift = False
        output_control_command.gear = 1
        self._set_control_and_wait_until_applied(output_control_command)

    def _set_control_and_wait_until_applied(self, control_command):
        rate = self.create_rate(1.0)
        while not self._is_gear_equal(self._vehicle_status_msg.control, control_command):
            self._vehicle_control_command_publisher.publish(control_command)
            rate.sleep()

    def _is_gear_equal(self, one, another):
        return one.manual_gear_shift == another.manual_gear_shift and \
            one.gear == another.gear


def main(args=None):
    rclpy.init(args=args)
    carla_service = CarlaService()

    # spin in another thread, because node.rate is used inside main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(carla_service, ), daemon=True)
    spin_thread.start()

    carla_service.prepare_vehicle_control()

    rate = carla_service.create_rate(1.0)
    while rclpy.ok():
        rate.sleep()

    carla_service.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
