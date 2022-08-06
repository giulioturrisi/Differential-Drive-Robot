#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py for subinterface_name, subinterface in interface.subinterfaces.items():
    else if(in->type == "`interface.full_name + (subinterface_name if subinterface_name != 'Message' else '')`")
    {
        `subinterface.cpp_type` x;
        ROS2WriteOptions wo;
        write__`subinterface.cpp_type_normalized`(x, in->_.stackID, &wo);
    }
#py endfor
#py endfor
