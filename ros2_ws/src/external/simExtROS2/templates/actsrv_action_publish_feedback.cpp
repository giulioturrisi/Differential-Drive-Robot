#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionServerProxy->actionType == "`interface.full_name`")
    {
        auto gh = getGoalHandle<`interface.cpp_type`>(actionServerProxy, in->goalUUID);
        auto feedback = std::make_shared<`interface.feedback.cpp_type`>();
        read__`interface.feedback.cpp_type_normalized`(in->_.stackID, feedback.get(), &(actionServerProxy->rd_opt));
        gh->publish_feedback(feedback);
    }
#py endif
#py endfor
