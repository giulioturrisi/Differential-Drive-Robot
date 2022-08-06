#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionServerProxy->actionType == "`interface.full_name`")
    {
        auto gh = getGoalHandle<`interface.cpp_type`>(actionServerProxy, in->goalUUID);
        out->result = gh->is_active();
    }
#py endif
#py endfor
