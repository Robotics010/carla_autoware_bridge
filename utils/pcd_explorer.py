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
        endian = ">"
    else: 
        endian = "<"
    
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
    
    format = f'{endian}{struct_format*count}'
    number = struct.unpack(format, field_bytes)
    if count == 1:
        number = number[0]
    return number

def main():
    pcd_path = '/home/robo/carla-autoware-ws/src/carla_autoware_bridge/test/data/pcd.json'
    
    with open(pcd_path, 'r') as file_handler:
        pcd = json.load(file_handler)
    
    point_step = pcd['point_step']
    is_bigendian = pcd['is_bigendian']
    width = pcd['width']
    fields = pcd['fields']
    data = pcd['data']
    
    rings = set()
    for index in range(width):
        point_data = get_point_data_by_index(data, index, point_step)
        
        point = convert_point_data_to_dict(point_data, fields, is_bigendian)
        rings.add(point['ring'])
    print(f'rings: {rings}')

if __name__ == '__main__':
    main()