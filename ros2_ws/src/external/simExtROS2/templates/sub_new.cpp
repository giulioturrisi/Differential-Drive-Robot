#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
    else if(in->topicType == "`interface.full_name`")
    {
        auto cb = [=](const `interface.cpp_type`::SharedPtr msg) { ros_callback__`interface.cpp_type_normalized`(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<`interface.cpp_type`>(in->topicName, qos, cb);
    }
#py endif
#py endfor
