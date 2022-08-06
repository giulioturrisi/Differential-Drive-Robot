#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(in->actionType == "`interface.full_name`")
    {
        actionClientProxy->action_client = rclcpp_action::create_client<`interface.cpp_type`>(node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_logging_interface(), node->get_node_waitables_interface(), in->actionName);
    }
#py endif
#py endfor
