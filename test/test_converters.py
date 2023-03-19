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
