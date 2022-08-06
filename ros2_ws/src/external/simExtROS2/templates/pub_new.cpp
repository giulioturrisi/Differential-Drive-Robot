#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
    else if(in->topicType == "`interface.full_name`")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<`interface.cpp_type`>(in->topicName, qos);
    }
#py endif
#py endfor
