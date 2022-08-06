#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
    else if(in->serviceType == "`interface.full_name`")
    {
        auto cb = [=](const std::shared_ptr<rmw_request_id_t> request_header, const `interface.request.cpp_type`::SharedPtr req, `interface.response.cpp_type`::SharedPtr res) { ros_srv_callback__`interface.cpp_type_normalized`(request_header, req, res, serviceProxy); };
        serviceProxy->service = node->create_service<`interface.cpp_type`>(in->serviceName, cb);
    }
#py endif
#py endfor
