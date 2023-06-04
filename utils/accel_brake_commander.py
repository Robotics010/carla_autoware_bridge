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

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tier4_vehicle_msgs.msg import ActuationCommandStamped


class AccelBrakeCommander(Node):

    def __init__(self):
        super().__init__('accel_brake_commander')

        self.accel = 0.0
        self.brake = 0.0
        self._pub_actuation_cmd = self.create_publisher(
            ActuationCommandStamped, '/control/command/actuation_cmd', 1)
        timer_period = 0.02  # seconds
        self._timer = self.create_timer(timer_period, self._on_timer)

    def _on_timer(self):
        msg_actuation_cmd = ActuationCommandStamped()
        msg_actuation_cmd.actuation.steer_cmd = 0.0
        msg_actuation_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_actuation_cmd.header.frame_id = 'base_link'
        msg_actuation_cmd.actuation.accel_cmd = self.accel
        msg_actuation_cmd.actuation.brake_cmd = self.brake
        self._pub_actuation_cmd.publish(msg_actuation_cmd)


class TwistReader(Node):

    def __init__(self):
        super().__init__('twist_reader')
        self._longitudinal_velocity = 0.0

        self._sub_odometry = self.create_subscription(
            Odometry, '/carla/ego_vehicle/odometry',
            self._odometry_callback, 1)

    @property
    def velocity(self):
        return self._longitudinal_velocity

    def _odometry_callback(self, odometry_msg):
        self._longitudinal_velocity = odometry_msg.twist.twist.linear.x


def print_help():
    print(
        'Select acceleration/velocity/brake with "ax", "vy" and "bz"'
        ' (for example, a0.4)\n'
        'then press Enter to execute acceleration and brake test\n'
        'send q or press Ctrl+C to exit')


def main(args=None):
    rclpy.init(args=args)
    accel_brake_commander = AccelBrakeCommander()
    twist_reader = TwistReader()

    print_help()

    accel = 0.0
    brake = 0.0
    velocity = 0.0
    timeout = 20.0

    is_shutdown = False
    while not is_shutdown:
        print(f'a {accel:.2f}, v {velocity:.2f}, b {brake:.2f}, t {timeout:.0f}')
        command = input('command: ')

        if command == '':
            print(f'Go with accel: {accel} up to vel: {velocity}'
                  f' then brake: {brake} with timeout {timeout} s')

            current_velocity = 0.0
            accel_brake_commander.accel = accel
            accel_brake_commander.brake = 0.0
            start = accel_brake_commander.get_clock().now().to_msg().sec
            now = accel_brake_commander.get_clock().now().to_msg().sec
            while current_velocity < velocity and now - start < timeout:
                rclpy.spin_once(accel_brake_commander)
                rclpy.spin_once(twist_reader)
                current_velocity = twist_reader.velocity
                now = accel_brake_commander.get_clock().now().to_msg().sec
                print(timeout - (now - start))

            accel_brake_commander.accel = 0.0
            accel_brake_commander.brake = brake
            start = accel_brake_commander.get_clock().now().to_msg().sec
            while current_velocity > 0.001 and now - start < timeout:
                rclpy.spin_once(accel_brake_commander)
                rclpy.spin_once(twist_reader)
                current_velocity = twist_reader.velocity
                now = accel_brake_commander.get_clock().now().to_msg().sec
                print(timeout - (now - start))

            accel_brake_commander.accel = 0.0
            accel_brake_commander.brake = 0.0
            rclpy.spin_once(accel_brake_commander)
        elif command[0] == 'a':
            try:
                accel = float(command[1:])
            except ValueError:
                print(f'can not parse {command[1:]}')
        elif command[0] == 'v':
            try:
                velocity = float(command[1:])
            except ValueError:
                print(f'can not parse {command[1:]}')
        elif command[0] == 'b':
            try:
                brake = float(command[1:])
            except ValueError:
                print(f'can not parse {command[1:]}')
        elif command[0] == 't':
            try:
                timeout = float(command[1:])
            except ValueError:
                print(f'can not parse {command[1:]}')
        elif command[0] == 'q':
            is_shutdown = True
        else:
            print(f'can not parse {command}')
            print_help()

    accel_brake_commander.destroy_node()
    twist_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
