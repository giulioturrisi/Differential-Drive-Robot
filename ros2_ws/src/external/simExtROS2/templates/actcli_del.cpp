#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionClientProxy->actionType == "`interface.full_name`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp_action::Client<`interface.cpp_type`> > >(actionClientProxy->action_client);
        cli = nullptr;
    }
#py endif
#py endfor
