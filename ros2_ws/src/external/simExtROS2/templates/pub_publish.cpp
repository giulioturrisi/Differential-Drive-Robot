#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
    else if(publisherProxy->topicType == "`interface.full_name`")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<`interface.cpp_type`> > >(publisherProxy->publisher);
        `interface.cpp_type` msg;
        read__`interface.cpp_type_normalized`(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
#py endif
#py endfor
