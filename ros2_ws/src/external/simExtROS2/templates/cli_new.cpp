#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
    else if(in->serviceType == "`interface.full_name`")
    {
        clientProxy->client = node->create_client<`interface.cpp_type`>(in->serviceName);
    }
#py endif
#py endfor
