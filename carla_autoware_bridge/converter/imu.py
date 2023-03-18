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

from carla_autoware_bridge.converter.converter import Converter

from sensor_msgs.msg import Imu


class ImuConverter(Converter):

    def __init__(self, default_inbox=None, default_outbox=None) -> None:
        if default_inbox is not None:
            if not isinstance(default_inbox, Imu):
                raise TypeError(f'default inbox value can be {Imu} type only')
        if default_outbox is not None:
            if not isinstance(default_outbox, Imu):
                raise TypeError(f'default outbox value can be {Imu} type only')
        super().__init__(default_inbox=default_inbox, default_outbox=default_outbox)

    @property
    def inbox(self):
        return self._inbox

    @inbox.setter
    def inbox(self, value):
        if not isinstance(value, Imu):
            raise TypeError(f'inbox value can be {Imu} type only')
        self._inbox = value

    def convert(self):
        self._outbox = self._inbox
