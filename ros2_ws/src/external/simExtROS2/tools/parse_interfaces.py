from sys import argv, exit, stderr
import os
import re
import subprocess
from rosidl_runtime_py import get_interface_path
from rosidl_adapter.parser import parse_message_file, parse_service_file, parse_action_file, MessageSpecification, ServiceSpecification, ActionSpecification


ctype_builtin = {
    'bool':         'bool',
    'int8':         'int8_t',
    'uint8':        'uint8_t',
    'int16':        'int16_t',
    'uint16':       'uint16_t',
    'int32':        'int32_t',
    'uint32':       'uint32_t',
    'int64':        'int64_t',
    'uint64':       'uint64_t',
    'float32':      'float',
    'float64':      'double',
    'string':       'std::string',
    'wstring':      'std::wstring',
    'time':         'ros::Time',
    'duration':     'ros::Duration',
    'byte':         'uint8_t',
    'char':         'int8_t'
}

fast_write_types = {'int32': 'Int32', 'float32': 'Float', 'float64': 'Double'}

def camel_case_to_snake_case(x):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', x)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def monkey_patch_fields(obj):
    for i, field in enumerate(obj.fields):
        if field.type.is_primitive_type():
            field.__dict__['cpp_type'] = ctype_builtin[field.type.type]
            field.__dict__['cpp_type_normalized'] = field.type.type
        else:
            field.__dict__['cpp_type'] = '::'.join([field.type.pkg_name, 'msg', field.type.type])
            field.__dict__['cpp_type_normalized'] = '{}__msg__{}'.format(field.type.pkg_name, field.type.type)

def parse_interface(m):
    assert isinstance(m, str)
    pkg, tag, name = m.split('/')
    path = get_interface_path(m)
    if tag == 'msg':
        obj = parse_message_file(pkg, path)
        monkey_patch_fields(obj)
        obj.__dict__['subinterfaces'] = {'Message': obj}
    elif tag == 'srv':
        obj = parse_service_file(pkg, path)
        monkey_patch_fields(obj.request)
        monkey_patch_fields(obj.response)
        obj.__dict__['subinterfaces'] = {'Request': obj.request, 'Response': obj.response}
    elif tag == 'action':
        obj = parse_action_file(pkg, path)
        monkey_patch_fields(obj.goal)
        monkey_patch_fields(obj.feedback)
        monkey_patch_fields(obj.result)
        obj.__dict__['subinterfaces'] = {'Goal': obj.goal, 'Feedback': obj.feedback, 'Result': obj.result}
    for subinterface_name, subinterface in obj.subinterfaces.items():
        subinterface.__dict__['cpp_type_normalized'] = '{}/{}'.format(m, subinterface_name).replace('/', '__')
        subinterface.__dict__['cpp_type'] = '::'.join([pkg, tag, name, subinterface_name])
    obj.__dict__['tag'] = tag
    obj.__dict__['full_name'] = m
    obj.__dict__['cpp_include'] = '{}/{}/{}.hpp'.format(pkg, tag, camel_case_to_snake_case(name))
    obj.__dict__['cpp_type_normalized'] = m.replace('/', '__')
    obj.__dict__['cpp_type'] = '::'.join([pkg, tag, name])
    return obj

def parse_interfaces(interfaces_file):
    interfaces_list = set()
    with open(interfaces_file) as f:
        for line in f:
            line = re.sub('#.*$', '', line).strip()
            if not line: continue
            interfaces_list.add(line)
    interfaces = {}
    for interface_name in sorted(interfaces_list):
        interfaces[interface_name] = parse_interface(interface_name)
    return interfaces
