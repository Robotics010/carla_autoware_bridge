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

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2


def convert_poincloud_to_dict(pointcloud: PointCloud2) -> dict:
    output = {
        'header': {
            'stamp': {
                'sec': pointcloud.header.stamp.sec,
                'nanosec': pointcloud.header.stamp.nanosec,
                },
            'frame_id': pointcloud.header.frame_id,
        },
        'height': pointcloud.height,
        'width': pointcloud.width,
        'is_bigendian': pointcloud.is_bigendian,
        'point_step': pointcloud.point_step,
        'row_step': pointcloud.row_step,
        'is_dense': pointcloud.is_dense,
    }

    fields = []
    for field in pointcloud.fields:
        fields.append({
            'name': field.name,
            'offset': field.offset,
            'datatype': field.datatype,
            'count': field.count,
        })
    output['fields'] = fields

    data = []
    for byte in pointcloud.data:
        data.append(byte)
    output['data'] = data

    return output


def main():
    bag_path = '/home/robo/rosbag/rosbag2_2023_04_07-02_30_39'
    pcd_path = '/home/robo/carla-autoware-ws/src/carla_autoware_bridge/test/data/pcd.json'
    pcd_ex_path = '/home/robo/carla-autoware-ws/src/carla_autoware_bridge/test/data/pcd_ex.json'
    pcd_carla_path = ('/home/robo/carla-autoware-ws/src/carla_autoware_bridge/'
                      'test/data/pcd_carla.json')

    serialization_format = 'cdr'
    storage_id = 'sqlite3'
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic == '/sensing/lidar/top/pointcloud_raw':
            assert isinstance(msg, PointCloud2)
            msg_dict = convert_poincloud_to_dict(msg)
            with open(pcd_path, 'w') as file_handler:
                json.dump(msg_dict, file_handler, indent=2)
        elif topic == '/sensing/lidar/top/pointcloud_raw_ex':
            assert isinstance(msg, PointCloud2)
            msg_dict = convert_poincloud_to_dict(msg)
            with open(pcd_ex_path, 'w') as file_handler:
                json.dump(msg_dict, file_handler, indent=2)
        elif topic == '/carla/ego_vehicle/lidar':
            assert isinstance(msg, PointCloud2)
            msg_dict = convert_poincloud_to_dict(msg)
            with open(pcd_carla_path, 'w') as file_handler:
                json.dump(msg_dict, file_handler, indent=2)


if __name__ == '__main__':
    main()
