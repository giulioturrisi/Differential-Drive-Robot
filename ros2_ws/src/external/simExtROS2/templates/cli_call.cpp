#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
    else if(clientProxy->serviceType == "`interface.full_name`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<`interface.cpp_type`> > >(clientProxy->client);
        auto req = std::make_shared<`interface.request.cpp_type`>();
        read__`interface.request.cpp_type_normalized`(in->_.stackID, req.get(), &(clientProxy->rd_opt));
        auto result = cli->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = result.get();
            write__`interface.response.cpp_type_normalized`(*resp, in->_.stackID, &(clientProxy->wr_opt));
        }
        else
        {
            throw sim::exception("failed to call service `interface.full_name`");
        }
    }
#py endif
#py endfor
