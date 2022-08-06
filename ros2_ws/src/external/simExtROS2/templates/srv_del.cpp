#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
    else if(serviceProxy->serviceType == "`interface.full_name`")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp::Service<`interface.cpp_type`> > >(serviceProxy->service);
        srv = nullptr;
    }
#py endif
#py endfor
