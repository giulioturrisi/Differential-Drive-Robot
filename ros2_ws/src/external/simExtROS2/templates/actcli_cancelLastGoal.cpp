#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionClientProxy->actionType == "`interface.full_name`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp_action::Client<`interface.cpp_type`> > >(actionClientProxy->action_client);
        auto gh = boost::any_cast< rclcpp_action::ClientGoalHandle<`interface.cpp_type`>::SharedPtr >(actionClientProxy->last_goal_handle);
        auto cancel_result_future = cli->async_cancel_goal(gh);
        if(rclcpp::spin_until_future_complete(node, cancel_result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            out->success = false;
        }
        else
        {
            out->success = true;
        }
    }
#py endif
#py endfor
