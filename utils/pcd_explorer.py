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

import json
import struct


def get_point_data_by_index(data, index, point_step):
    from_index = index * point_step
    to_index = from_index + point_step
    return data[from_index:to_index]


def convert_point_data_to_dict(point_data, fields, is_bigendian):
    point = {}
    for field in fields:
        field_name = field['name']
        offset = field['offset']
        datatype = field['datatype']
        count = field['count']
        bytes_count = get_bytes_count_from_datatype(datatype)
        field_data = point_data[offset:offset+bytes_count*count]
        field_value = convert_field_data(field_data, datatype, count, is_bigendian)
        point[field_name] = field_value
    return point


def get_bytes_count_from_datatype(datatype):
    if datatype >= 1 and datatype <= 2:
        bytes_count = 1
    elif datatype >= 3 and datatype <= 4:
        bytes_count = 2
    elif datatype >= 5 and datatype <= 7:
        bytes_count = 4
    elif datatype == 8:
        bytes_count = 8
    else:
        raise RuntimeError(f'{datatype} datatype unimplemented')
    return bytes_count


def convert_field_data(field_data, datatype, count, is_bigendian):
    field_bytes = bytes(field_data)
    if is_bigendian:
        endian = '>'
    else:
        endian = '<'

    datatype_to_struct_format = {
        1: 'b',
        2: 'B',
        3: 'h',
        4: 'H',
        5: 'i',
        6: 'I',
        7: 'f',
        8: 'd',
    }
    struct_format = datatype_to_struct_format[datatype]

    struct_format = f'{endian}{struct_format*count}'
    number = struct.unpack(struct_format, field_bytes)
    if count == 1:
        number = number[0]
    return number


def main():
    pcd_path = '/home/robo/carla-autoware-ws/src/carla_autoware_bridge/test/data/pcd_ex.json'

    with open(pcd_path, 'r') as file_handler:
        pcd = json.load(file_handler)

    point_step = pcd['point_step']
    is_bigendian = pcd['is_bigendian']
    # width = pcd['width']
    fields = pcd['fields']
    data = pcd['data']

    index = 20000
    point_data = get_point_data_by_index(data, index, point_step)
    print(f'point_data: {point_data}')
    point = convert_point_data_to_dict(point_data, fields, is_bigendian)
    print(f'point: {point}')


if __name__ == '__main__':
    main()
