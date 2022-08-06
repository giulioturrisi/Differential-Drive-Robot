#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionServerProxy->actionType == "`interface.full_name`")
    {
        auto gh = getGoalHandle<`interface.cpp_type`>(actionServerProxy, in->goalUUID);
        auto result = std::make_shared<`interface.result.cpp_type`>();
        read__`interface.result.cpp_type_normalized`(in->_.stackID, result.get(), &(actionServerProxy->rd_opt));
        gh->succeed(result);
    }
#py endif
#py endfor
