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

from autoware_auto_vehicle_msgs.msg import VelocityReport
from carla_autoware_bridge.converter.converter import Converter
from nav_msgs.msg import Odometry


class VelocityReportConverter(Converter):

    def _convert(self):
        if not isinstance(self._inbox, Odometry):
            raise RuntimeError(f'Input must be {Odometry}!')

        # Convert from left-handed Unreal coordinate frame
        # to right-handed ROS2 coordinate frame
        yaw_rate = self._inbox.twist.twist.angular.z
        longitudinal_velocity = self._inbox.twist.twist.linear.x
        lateral_velocity = self._inbox.twist.twist.linear.y

        output_velocity_report = VelocityReport()
        output_velocity_report.heading_rate = -yaw_rate
        output_velocity_report.longitudinal_velocity = longitudinal_velocity
        output_velocity_report.lateral_velocity = -lateral_velocity
        self._outbox = output_velocity_report
