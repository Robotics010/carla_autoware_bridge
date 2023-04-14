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
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


class LidarExtendedConverter(Converter):

    def _convert(self):
        if not isinstance(self._inbox, PointCloud2):
            raise RuntimeError(f'Input must be {PointCloud2}!')

        input_pointcloud = self._inbox

        pointcloud_ex = PointCloud2()
        pointcloud_ex.header.frame_id = input_pointcloud.header.frame_id
        pointcloud_ex.header.stamp.sec = input_pointcloud.header.stamp.sec
        pointcloud_ex.header.stamp.nanosec = input_pointcloud.header.stamp.nanosec
        pointcloud_ex.height = input_pointcloud.height
        pointcloud_ex.width = input_pointcloud.width
        pointcloud_ex.is_bigendian = input_pointcloud.is_bigendian
        pointcloud_ex.point_step = input_pointcloud.point_step + 8  # two of float32 numbers
        pointcloud_ex.row_step = input_pointcloud.row_step
        pointcloud_ex.is_dense = input_pointcloud.is_dense

        data = np.fromstring(
            bytes(input_pointcloud.data),
            dtype=self._convert_fields_to_dtype(input_pointcloud.fields))

        pointcloud_ex.fields = input_pointcloud.fields
        self._add_field(pointcloud_ex.fields,
                        PointField(name='azimuth', datatype=PointField.FLOAT32, count=1))
        self._add_field(pointcloud_ex.fields,
                        PointField(name='distance', datatype=PointField.FLOAT32, count=1))

        data = self._calculate_and_add_azimuth_and_distance(data)

        pointcloud_ex.data = self._create_pointcloud_data(
            data, pointcloud_ex.fields, pointcloud_ex.is_bigendian)

        self._outbox = pointcloud_ex

    def _add_field(self, fields, new_field):
        last_field = fields[-1]
        last_bytes_count = self._get_bytes_count_from_datatype(last_field.datatype)

        new_field.offset = last_field.offset + last_bytes_count * last_field.count

        fields.append(new_field)

    def _get_bytes_count_from_datatype(self, datatype):
        if datatype >= PointField.INT8 and datatype <= PointField.UINT8:
            bytes_count = 1
        elif datatype >= PointField.INT16 and datatype <= PointField.UINT16:
            bytes_count = 2
        elif datatype >= PointField.INT32 and datatype <= PointField.FLOAT32:
            bytes_count = 4
        elif datatype == PointField.FLOAT64:
            bytes_count = 8
        else:
            raise RuntimeError(f'{datatype} datatype unimplemented')
        return bytes_count

    def _convert_fields_to_dtype(self, fields):
        types = []
        for field in fields:
            types.append((
                field.name,
                self._convert_datatype_to_dtype(field.datatype)
            ))
        return np.dtype(types)

    def _convert_fields_to_struct_format(self, fields, is_bigendian):
        struct_format = ''
        for field in fields:
            struct_format += self._convert_datatype_to_struct_format(field.datatype)

        return struct_format

    def _convert_datatype_to_dtype(self, datatype):
        if datatype == PointField.INT8:
            return np.int8
        elif datatype == PointField.UINT8:
            return np.uint8
        elif datatype == PointField.INT16:
            return np.int16
        elif datatype == PointField.UINT16:
            return np.uint16
        elif datatype == PointField.INT32:
            return np.int32
        elif datatype == PointField.UINT32:
            return np.uint32
        elif datatype == PointField.FLOAT32:
            return np.float32
        elif datatype == PointField.FLOAT64:
            return np.float64

    def _convert_datatype_to_struct_format(self, datatype):
        if datatype == PointField.INT8:
            return 'b'
        elif datatype == PointField.UINT8:
            return 'B'
        elif datatype == PointField.INT16:
            return 'h'
        elif datatype == PointField.UINT16:
            return 'H'
        elif datatype == PointField.INT32:
            return 'i'
        elif datatype == PointField.UINT32:
            return 'I'
        elif datatype == PointField.FLOAT32:
            return 'f'
        elif datatype == PointField.FLOAT64:
            return 'd'

    def _calculate_and_add_azimuth_and_distance(self, data):
        new_dtype = np.dtype(data.dtype.descr + [('azimuth', '<f4'), ('distance', '<f4')])
        new_data = np.empty(data.shape, dtype=new_dtype)
        for description in data.dtype.descr:
            field_name = description[0]
            new_data[field_name] = data[field_name]

        for row in new_data:
            row['azimuth'] = np.arctan2(row['x'], row['y'])
            row['distance'] = np.sqrt(
                np.square(row['x']) + np.square(row['y']) + np.square(row['z']))

        return new_data

    def _create_pointcloud_data(self, data, fields, is_bigendian):
        return data.tobytes('C')
