#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
    else if(clientProxy->serviceType == "`interface.full_name`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<`interface.cpp_type`> > >(clientProxy->client);
        cli = nullptr;
    }
#py endif
#py endfor
