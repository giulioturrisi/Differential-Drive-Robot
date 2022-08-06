#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionServerProxy->actionType == "`interface.full_name`")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp_action::Server<`interface.cpp_type`> > >(actionServerProxy->action_server);
        srv = nullptr;
    }
#py endif
#py endfor
