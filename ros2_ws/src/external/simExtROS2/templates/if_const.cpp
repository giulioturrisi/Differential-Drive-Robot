#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py for subinterface_name, subinterface in interface.subinterfaces.items():
    else if(in->type == "`interface.full_name + (subinterface_name if subinterface_name != 'Message' else '')`")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
#py for constant in subinterface.constants:
        sim::pushStringOntoStack(in->_.stackID, "`constant.name`", 0);
        write__`constant.type`(`constant.value`, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
#py endfor
    }
#py endfor
#py endfor
