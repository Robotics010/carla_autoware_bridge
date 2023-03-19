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

from abc import ABC, abstractmethod


class Converter(ABC):

    def __init__(self, default_inbox=None, default_outbox=None) -> None:
        super().__init__()
        self._inbox = default_inbox
        self._outbox = default_outbox

    @property
    def inbox(self):
        return self._inbox

    @inbox.setter
    def inbox(self, value):
        self._inbox = value

    @property
    def outbox(self):
        if self._outbox is None:
            raise RuntimeError('Call convert before getting outbox!')
        return self._outbox

    def convert(self):
        if self._inbox is None:
            raise RuntimeError('Set inbox before calling convert!')
        self._convert()

    @abstractmethod
    def _convert(self):
        pass
