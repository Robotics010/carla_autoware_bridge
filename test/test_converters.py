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


import struct

from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from carla_autoware_bridge.converter.actuation_status import ActuationStatusConverter
from carla_autoware_bridge.converter.control_command import ControlCommandConverter
from carla_autoware_bridge.converter.fake import FakeConverter
from carla_autoware_bridge.converter.lidar_ex import LidarExtendedConverter
from carla_autoware_bridge.converter.steering_status import SteeringStatusConverter
from carla_autoware_bridge.converter.velocity_report import VelocityReportConverter
from carla_msgs.msg import (
    CarlaEgoVehicleControl,
    CarlaEgoVehicleStatus,
    CarlaEgoVehicleSteering)
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped


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


def test_velocity_report_convert():
    input_odometry = Odometry()
    angular_velocity = input_odometry.twist.twist.angular
    angular_velocity.z = -0.317
    linear_velocity = input_odometry.twist.twist.linear
    linear_velocity.x = 5.55
    linear_velocity.y = 0.77

    output_velocity_report = VelocityReport()
    output_velocity_report.heading_rate = 0.317
    output_velocity_report.longitudinal_velocity = 5.55
    output_velocity_report.lateral_velocity = -0.77

    vel_rep_converter = VelocityReportConverter()
    vel_rep_converter.inbox = input_odometry
    vel_rep_converter.convert()
    assert vel_rep_converter.outbox.longitudinal_velocity == \
        output_velocity_report.longitudinal_velocity
    assert vel_rep_converter.outbox.lateral_velocity == \
        output_velocity_report.lateral_velocity
    assert vel_rep_converter.outbox.heading_rate == output_velocity_report.heading_rate


def test_velocity_report_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    vel_rep_converter = VelocityReportConverter()
    vel_rep_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        vel_rep_converter.convert()


def test_left_steering_status_convert():
    fl_max_left_angle = -48.99
    fr_max_left_angle = -35.077
    average_max_left_angle = np.radians(fl_max_left_angle + fr_max_left_angle) / 2

    input_vehicle_steering = CarlaEgoVehicleSteering()
    input_vehicle_steering.steering_tire_angle = average_max_left_angle

    expected_steering_tire_angle = -average_max_left_angle
    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = expected_steering_tire_angle

    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_vehicle_steering
    steering_status_converter.convert()

    assert pytest.approx(steering_status_converter.outbox.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)


def test_right_steering_status_convert():
    fl_max_right_angle = 35.077
    fr_max_right_angle = 48.99
    average_max_right_angle = np.radians(fl_max_right_angle + fr_max_right_angle) / 2

    input_vehicle_steering = CarlaEgoVehicleSteering()
    input_vehicle_steering.steering_tire_angle = average_max_right_angle

    expected_steering_tire_angle = -average_max_right_angle
    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = expected_steering_tire_angle

    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_vehicle_steering
    steering_status_converter.convert()

    assert pytest.approx(steering_status_converter.outbox.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)


def test_center_steering_status_convert():
    input_vehicle_steering = CarlaEgoVehicleSteering()
    input_vehicle_steering.steering_tire_angle = 0.0

    expected_steering_status = SteeringReport()
    expected_steering_status.steering_tire_angle = 0.0

    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_vehicle_steering
    steering_status_converter.convert()

    assert pytest.approx(steering_status_converter.outbox.steering_tire_angle) == \
        pytest.approx(expected_steering_status.steering_tire_angle)


def test_steering_status_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    steering_status_converter = SteeringStatusConverter()
    steering_status_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        steering_status_converter.convert()


def test_throttle_control_command():
    input_control_command = ActuationCommandStamped()
    input_control_command.actuation.accel_cmd = 0.301

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.throttle = 0.301
    expected_control_command.brake = 0.0

    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_control_command
    control_command_converter.convert()

    assert pytest.approx(control_command_converter.outbox.throttle) == \
        pytest.approx(expected_control_command.throttle)
    assert pytest.approx(control_command_converter.outbox.brake) == \
        pytest.approx(expected_control_command.brake)


def test_still_control_command():
    input_control_command = ActuationCommandStamped()
    input_control_command.actuation.accel_cmd = 0.0

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.throttle = 0.0
    expected_control_command.brake = 0.0

    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_control_command
    control_command_converter.convert()

    assert pytest.approx(control_command_converter.outbox.throttle) == \
        pytest.approx(expected_control_command.throttle)
    assert pytest.approx(control_command_converter.outbox.brake) == \
        pytest.approx(expected_control_command.brake)


def test_brake_control_command():
    input_control_command = ActuationCommandStamped()
    input_control_command.actuation.brake_cmd = 0.205

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.throttle = 0.0
    expected_control_command.brake = 0.205

    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_control_command
    control_command_converter.convert()

    assert pytest.approx(control_command_converter.outbox.throttle) == \
        pytest.approx(expected_control_command.throttle)
    assert pytest.approx(control_command_converter.outbox.brake) == \
        pytest.approx(expected_control_command.brake)


def test_steering_left_control_command():
    input_control_command = ActuationCommandStamped()
    input_control_command.actuation.steer_cmd = -0.299

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.steer = 0.407566

    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_control_command
    control_command_converter.convert()

    assert pytest.approx(control_command_converter.outbox.steer) == \
        pytest.approx(expected_control_command.steer)


def test_steering_right_control_command():
    input_control_command = ActuationCommandStamped()
    input_control_command.actuation.steer_cmd = 0.299

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.steer = -0.407566

    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_control_command
    control_command_converter.convert()

    assert pytest.approx(control_command_converter.outbox.steer) == \
        pytest.approx(expected_control_command.steer)


def test_gear_control_command():
    input_control_command = ActuationCommandStamped()

    expected_control_command = CarlaEgoVehicleControl()
    expected_control_command.manual_gear_shift = False

    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_control_command
    control_command_converter.convert()

    assert pytest.approx(control_command_converter.outbox.manual_gear_shift) == \
        pytest.approx(expected_control_command.manual_gear_shift)


def test_control_comman_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    control_command_converter = ControlCommandConverter()
    control_command_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        control_command_converter.convert()


def test_lidar_ex_conversion():
    frame_id = 'velodyne_top'
    sec = 9
    nanosec = 89999796
    height = 1
    width = 27097
    is_bigendian = False
    point_step = 20
    row_step = 650328
    is_dense = True

    input_pointcloud = PointCloud2()
    input_pointcloud.header.frame_id = frame_id
    input_pointcloud.header.stamp.sec = sec
    input_pointcloud.header.stamp.nanosec = nanosec
    input_pointcloud.height = height
    input_pointcloud.width = width
    input_pointcloud.is_bigendian = is_bigendian
    input_pointcloud.point_step = point_step
    input_pointcloud.row_step = row_step
    input_pointcloud.is_dense = is_dense
    input_pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1)
        ]
    input_pointcloud.data = [
        128, 225, 251, 192,
        0, 0, 160, 183,
        72, 73, 5, 192,
        0, 0, 200, 66,
        1, 0,
        192, 190, 19, 193,
        0, 0, 144, 56,
        86, 165, 6, 192,
        0, 0, 200, 66,
        2, 0,
        ]

    expected_pointcloud_ex = PointCloud2()
    expected_pointcloud_ex.header.frame_id = frame_id
    expected_pointcloud_ex.header.stamp.sec = sec
    expected_pointcloud_ex.header.stamp.nanosec = nanosec
    expected_pointcloud_ex.height = height
    expected_pointcloud_ex.width = width
    expected_pointcloud_ex.is_bigendian = is_bigendian
    expected_pointcloud_ex.point_step = point_step + 8  # two of float32 numbers
    expected_pointcloud_ex.row_step = row_step
    expected_pointcloud_ex.is_dense = is_dense

    last_input_field = input_pointcloud.fields[-1]
    last_offset = last_input_field.offset
    last_datatype = last_input_field.datatype
    last_count = last_input_field.count
    last_bytes_count = get_bytes_count_from_datatype(last_datatype)

    expected_azimuth_name = 'azimuth'
    expected_azimuth_offset = last_offset + last_bytes_count * last_count

    expected_azimuth_datatype = PointField.FLOAT32
    expected_azimuth_count = 1
    expected_azimuth_bytes_count = get_bytes_count_from_datatype(expected_azimuth_datatype)

    expected_distance_name = 'distance'
    expected_distance_offset = expected_azimuth_offset \
        + expected_azimuth_bytes_count * expected_azimuth_count
    expected_distance_datatype = PointField.FLOAT32
    expected_distance_count = 1

    expected_fields_length = len(input_pointcloud.fields) + 2

    index = 0
    point_data = get_point_data_by_index(input_pointcloud.data, index, point_step)
    point = convert_point_data_to_dict(point_data, input_pointcloud.fields, is_bigendian)
    a = np.array((point['x'], point['y'], point['z']))
    b = np.array((0, 0, 0))
    expected_azimuth = np.arctan2(point['x'], point['y'])
    expected_distance = np.linalg.norm(a-b)

    lidar_ex_converter = LidarExtendedConverter()
    lidar_ex_converter.inbox = input_pointcloud
    lidar_ex_converter.convert()

    assert lidar_ex_converter.outbox.header.frame_id == \
        expected_pointcloud_ex.header.frame_id
    assert lidar_ex_converter.outbox.header.stamp.sec == \
        expected_pointcloud_ex.header.stamp.sec
    assert lidar_ex_converter.outbox.header.stamp.nanosec == \
        expected_pointcloud_ex.header.stamp.nanosec
    assert lidar_ex_converter.outbox.height == \
        expected_pointcloud_ex.height
    assert lidar_ex_converter.outbox.width == \
        expected_pointcloud_ex.width
    assert lidar_ex_converter.outbox.is_bigendian == \
        expected_pointcloud_ex.is_bigendian
    assert lidar_ex_converter.outbox.point_step == \
        expected_pointcloud_ex.point_step
    assert lidar_ex_converter.outbox.row_step == \
        expected_pointcloud_ex.row_step
    assert lidar_ex_converter.outbox.is_dense == \
        expected_pointcloud_ex.is_dense

    outbox_azimuth = lidar_ex_converter.outbox.fields[5]
    assert outbox_azimuth.name == expected_azimuth_name
    assert outbox_azimuth.offset == expected_azimuth_offset
    assert outbox_azimuth.datatype == expected_azimuth_datatype
    assert outbox_azimuth.count == expected_azimuth_count

    outbox_distance = lidar_ex_converter.outbox.fields[6]
    assert outbox_distance.name == expected_distance_name
    assert outbox_distance.offset == expected_distance_offset
    assert outbox_distance.datatype == expected_distance_datatype
    assert outbox_distance.count == expected_distance_count

    assert len(lidar_ex_converter.outbox.fields) == expected_fields_length

    index = 0
    print(f'outbox.data: {lidar_ex_converter.outbox.data}')
    point_data = get_point_data_by_index(
        lidar_ex_converter.outbox.data, index, lidar_ex_converter.outbox.point_step)
    print(f'point_data: {point_data}')
    point = convert_point_data_to_dict(point_data, lidar_ex_converter.outbox.fields,
                                       is_bigendian)
    print(f'point: {point}')
    assert pytest.approx(expected_distance) == pytest.approx(point['distance'])
    assert pytest.approx(expected_azimuth) == pytest.approx(point['azimuth'])


def get_bytes_count_from_datatype(datatype):
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


def get_point_data_by_index(data, index, point_step):
    from_index = index * point_step
    to_index = from_index + point_step
    return data[from_index:to_index]


def convert_point_data_to_dict(point_data, fields, is_bigendian):
    point = {}
    for field in fields:
        field_name = field.name
        offset = field.offset
        datatype = field.datatype
        count = field.count
        bytes_count = get_bytes_count_from_datatype(datatype)
        field_data = point_data[offset:offset+bytes_count*count]
        field_value = convert_field_data(field_data, datatype, count, is_bigendian)
        print(f'{field_name}) {field_data}, {field_value}')
        point[field_name] = field_value
    return point


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


def test_lidar_ex_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    lidar_ex_converter = LidarExtendedConverter()
    lidar_ex_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        lidar_ex_converter.convert()

def test_actuation_status():
    
    input_vehicle_status = CarlaEgoVehicleStatus()
    input_vehicle_status.control.throttle = 0.4
    input_vehicle_status.control.brake = 0.0
    input_vehicle_status.control.steer = -0.1

    expected_actuation_status = ActuationStatusStamped()
    expected_actuation_status.status.accel_status = 0.4
    expected_actuation_status.status.brake_status = 0.0
    expected_actuation_status.status.steer_status = -0.1

    actuation_status_converter = ActuationStatusConverter()
    actuation_status_converter.inbox = input_vehicle_status
    actuation_status_converter.convert()

    assert pytest.approx(actuation_status_converter.outbox.status.accel_status) == \
        pytest.approx(expected_actuation_status.status.accel_status)
    assert pytest.approx(actuation_status_converter.outbox.status.brake_status) == \
        pytest.approx(expected_actuation_status.status.brake_status)
    assert pytest.approx(actuation_status_converter.outbox.status.steer_status) == \
        pytest.approx(expected_actuation_status.status.steer_status)


def test_actuation_status_invalid_input():
    class UnexpectedInput():
        pass
    input_invalid = UnexpectedInput()
    actuation_status_converter = ActuationStatusConverter()
    actuation_status_converter.inbox = input_invalid
    with pytest.raises(RuntimeError):
        actuation_status_converter.convert()
