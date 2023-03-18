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

from carla_autoware_bridge.converter.fake import FakeConverter
from carla_autoware_bridge.converter.imu import ImuConverter

import pytest

from sensor_msgs.msg import Imu


def test_fake_default_inbox():
    fake_converter = FakeConverter()
    assert fake_converter.inbox is None


def test_fake_set_default_inbox():
    default_value = 10
    fake_converter = FakeConverter(default_inbox=default_value)
    assert fake_converter.inbox == default_value


def test_fake_default_outbox():
    fake_converter = FakeConverter()
    assert fake_converter.outbox is None


def test_fake_set_default_outbox():
    default_value = 10
    fake_converter = FakeConverter(default_outbox=default_value)
    assert fake_converter.outbox == default_value


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


def test_imu_default_input_type():
    default_value = Imu()
    imu_converter = ImuConverter(default_inbox=default_value)
    assert imu_converter.inbox == default_value


def test_imu_default_raise_wrong_input_type():
    default_non_imu_msg = 10
    with pytest.raises(TypeError):
        imu_converter = ImuConverter(default_inbox=default_non_imu_msg)
        imu_converter


def test_imu_default_output_type():
    default_value = Imu()
    imu_converter = ImuConverter(default_outbox=default_value)
    assert imu_converter.outbox == default_value


def test_imu_default_raise_wrong_output_type():
    default_non_imu_msg = 10
    with pytest.raises(TypeError):
        imu_converter = ImuConverter(default_outbox=default_non_imu_msg)
        imu_converter


def test_imu_no_raise_imu_input_type():
    input_imu_msg = Imu()
    imu_converter = ImuConverter()
    imu_converter.inbox = input_imu_msg


def test_imu_raise_wrong_input_type():
    input_non_imu_msg = 10
    imu_converter = ImuConverter()
    with pytest.raises(TypeError):
        imu_converter.inbox = input_non_imu_msg


def test_imu_angular_velocity_conversion():
    input_imu_msg = Imu()
    expected_output_imu_msg = Imu()
    imu_converter = ImuConverter()
    imu_converter.inbox = input_imu_msg
    imu_converter.convert()
    assert imu_converter.outbox == expected_output_imu_msg
