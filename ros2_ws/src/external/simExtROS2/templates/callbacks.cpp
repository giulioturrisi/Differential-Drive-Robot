#include <callbacks.h>
#include <simLib.h>
#include <stubs.h>
#include <cstring>

#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py for subinterface_name, subinterface in interface.subinterfaces.items():
void write__`subinterface.cpp_type_normalized`(const `subinterface.cpp_type`& msg, int stack, const ROS2WriteOptions *opt)
{
    try
    {
        sim::pushTableOntoStack(stack);
#py for field in subinterface.fields:
#py if field.type.is_array:
#py if field.type.is_primitive_type() and field.type.type in fast_write_types:
        try
        {
            // write field '`field.name`' (using fast specialized function)
            sim::pushStringOntoStack(stack, "`field.name`", 0);
            sim::push`fast_write_types[field.type.type]`TableOntoStack(stack, &(msg.`field.name`[0]), msg.`field.name`.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py elif field.type.is_primitive_type() and field.type.type == 'uint8':
        try
        {
            // write field '`field.name`' (using fast specialized function)
            sim::pushStringOntoStack(stack, "`field.name`", 0);
            if(opt && opt->uint8array_as_string)
                sim::pushStringOntoStack(stack, (simChar*)&(msg.`field.name`[0]), msg.`field.name`.size());
            else
                sim::pushUInt8TableOntoStack(stack, &(msg.`field.name`[0]), msg.`field.name`.size());
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py else:
        try
        {
            // write field '`field.name`'
            sim::pushStringOntoStack(stack, "`field.name`", 0);
            sim::pushTableOntoStack(stack);
            for(int i = 0; i < msg.`field.name`.size(); i++)
            {
                write__int32(i + 1, stack, opt);
                write__`field.cpp_type_normalized`(msg.`field.name`[i], stack, opt);
                sim::insertDataIntoStackTable(stack);
            }
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py endif
#py else:
        try
        {
            // write field '`field.name`'
            sim::pushStringOntoStack(stack, "`field.name`", 0);
            write__`field.cpp_type_normalized`(msg.`field.name`, stack, opt);
            sim::insertDataIntoStackTable(stack);
        }
        catch(std::exception &ex)
        {
            std::string msg = "field '`field.name`': ";
            msg += ex.what();
            throw sim::exception(msg);
        }
#py endif
#py endfor
    }
    catch(std::exception &ex)
    {
        std::string msg = "write__`subinterface.cpp_type_normalized`: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

void read__`subinterface.cpp_type_normalized`(int stack, `subinterface.cpp_type` *msg, const ROS2ReadOptions *opt)
{
    try
    {
        int r = sim::getStackTableInfo(stack, 0);
        if(r != sim_stack_table_map && r != sim_stack_table_empty)
            throw sim::exception("expected a table");

        int oldsz = sim::getStackSize(stack);
        sim::unfoldStackTable(stack);
        int numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;

        char *str;
        int strSz;

        while(numItems >= 1)
        {
            sim::moveStackItemToTop(stack, oldsz - 1); // move key to top
            if((str = sim::getStackStringValue(stack, &strSz)) != NULL && strSz > 0)
            {
                sim::popStackItem(stack, 1);

                sim::moveStackItemToTop(stack, oldsz - 1); // move value to top

                if(0) {}
#py for field in subinterface.fields:
#py if field.type.is_array:
#py if field.type.is_primitive_type() and field.type.type in fast_write_types:
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        // read field '`field.name`' (using fast specialized function)
                        int sz = sim::getStackTableInfo(stack, 0);
                        if(sz < 0)
                            throw sim::exception("expected array");
                        if(sim::getStackTableInfo(stack, 2) != 1)
                            throw sim::exception("fast_write_type reader exception #1");
#py if field.type.array_size:
                        // field has fixed size -> no need to reserve space into vector
#py else:
                        msg->`field.name`.resize(sz);
#py endif
                        sim::getStack`fast_write_types[field.type.type]`Table(stack, &(msg->`field.name`[0]), sz);
                        sim::popStackItem(stack, 1);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py elif field.type.is_primitive_type() and field.type.type == 'uint8':
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        if(opt && opt->uint8array_as_string)
                        {
                            // read field '`field.name`' (uint8[]) as string
                            simChar *str;
                            simInt sz;
                            if((str = sim::getStackStringValue(stack, &sz)) != NULL && sz > 0)
                            {
                                /*
                                 * XXX: if an alternative version of simGetStackStringValue woudl exist
                                 * working on an externally allocated buffer, we won't need this memcpy:
                                 */
#py if field.type.array_size:
                                // field has fixed size -> no need to reserve space into vector
#py else:
                                msg->`field.name`.resize(sz);
#py endif
                                std::memcpy(&(msg->`field.name`[0]), str, sz);
                                sim::releaseBuffer(str);
                            }
                            else throw sim::exception("string read error when trying to read uint8[]");
                        }
                        else
			{
                            // read field '`field.name`' (using fast specialized function)
                            int sz = sim::getStackTableInfo(stack, 0);
                            if(sz < 0)
                                throw sim::exception("expected uint8 array");
                            if(sim::getStackTableInfo(stack, 2) != 1)
                                throw sim::exception("fast_write_type uint8[] reader exception #1");
#py if field.type.array_size:
                            // field has fixed size -> no need to reserve space into vector
#py else:
                            msg->`field.name`.resize(sz);
#py endif
                            sim::getStackUInt8Table(stack, &(msg->`field.name`[0]), sz);
                            sim::popStackItem(stack, 1);
			}
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py else:
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        // read field '`field.name`'
                        if(sim::getStackTableInfo(stack, 0) < 0)
                            throw sim::exception("expected array");
                        int oldsz1 = sim::getStackSize(stack);
                        sim::unfoldStackTable(stack);
                        int numItems1 = (sim::getStackSize(stack) - oldsz1 + 1) / 2;
                        for(int i = 0; i < numItems1; i++)
                        {
                            sim::moveStackItemToTop(stack, oldsz1 - 1); // move key to top
                            int j;
                            read__int32(stack, &j, opt);
                            sim::moveStackItemToTop(stack, oldsz1 - 1); // move value to top
                            `field.cpp_type` v;
                            read__`field.cpp_type_normalized`(stack, &v, opt);
#py if field.type.array_size:
                            msg->`field.name`[i] = (v);
#py else:
                            msg->`field.name`.push_back(v);
#py endif
                        }
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py endif
#py else:
                else if(strcmp(str, "`field.name`") == 0)
                {
                    try
                    {
                        // read field '`field.name`'
                        read__`field.cpp_type_normalized`(stack, &(msg->`field.name`), opt);
                    }
                    catch(std::exception &ex)
                    {
                        std::string msg = "field `field.name`: ";
                        msg += ex.what();
                        throw sim::exception(msg);
                    }
                }
#py endif
#py endfor
                else
                {
                    std::string msg = "unexpected key: ";
                    msg += str;
                    throw sim::exception(msg);
                }

                simReleaseBuffer(str);
            }
            else
            {
                throw sim::exception("malformed table (bad key type)");
            }

            numItems = (sim::getStackSize(stack) - oldsz + 1) / 2;
        }
    }
    catch(std::exception &ex)
    {
        std::string msg = "read__`subinterface.cpp_type_normalized`: ";
        msg += ex.what();
        throw sim::exception(msg);
    }
}

#py endfor
#py endfor

#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
void ros_callback__`interface.cpp_type_normalized`(const `interface.cpp_type`::SharedPtr msg, SubscriptionProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__`interface.cpp_type_normalized`(*msg, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->topicCallback.scriptId, proxy->topicCallback.name.c_str(), stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_callback__`interface.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(proxy->topicCallback.name.c_str(), msg.c_str());
    }
}

#py endif
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
bool ros_srv_callback__`interface.cpp_type_normalized`(const std::shared_ptr<rmw_request_id_t> request_header, const `interface.request.cpp_type`::SharedPtr req, `interface.response.cpp_type`::SharedPtr res, ServiceProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = sim::createStack();
        write__`interface.request.cpp_type_normalized`(*req, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__`interface.response.cpp_type_normalized`(stack, res.get(), &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
        return true;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__`interface.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(proxy->serviceCallback.name.c_str(), msg.c_str());
        return false;
    }
}

#py endif
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
void ros_action_callback__`interface.feedback.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.feedback.cpp_type` *feedback, ActionClientProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__`interface.feedback.cpp_type_normalized`(*feedback, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__`interface.feedback.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(callback, msg.c_str());
    }
}

void ros_action_callback__`interface.result.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, int action_result_code, const `interface.result.cpp_type`::SharedPtr result, ActionClientProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__int32(action_result_code, stack, &(proxy->wr_opt));
        write__`interface.result.cpp_type_normalized`(*result, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__`interface.result.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(callback, msg.c_str());
    }
}

rclcpp_action::GoalResponse ros_action_callback__handle_goal__`interface.goal.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.goal.cpp_type` *goal, ActionServerProxy *proxy)
{
    int stack = -1;
    int ret = sim_ros2_goal_response_reject;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__`interface.goal.cpp_type_normalized`(*goal, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        read__int32(stack, &ret, &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__handle_goal__`interface.goal.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(callback, msg.c_str());
    }
    switch(ret)
    {
    case sim_ros2_goal_response_reject:
        return rclcpp_action::GoalResponse::REJECT;
    case sim_ros2_goal_response_accept_and_execute:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    case sim_ros2_goal_response_accept_and_defer:
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    default:
        sim::addLog(sim_verbosity_scripterrors, "invalid goal response");
        return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse ros_action_callback__handle_cancel__`interface.goal.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.goal.cpp_type` *goal, ActionServerProxy *proxy)
{
    int stack = -1;
    int ret = sim_ros2_cancel_response_reject;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__`interface.goal.cpp_type_normalized`(*goal, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        read__int32(stack, &ret, &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__handle_cancel__`interface.goal.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(callback, msg.c_str());
    }
    switch(ret)
    {
    case sim_ros2_cancel_response_reject:
        return rclcpp_action::CancelResponse::REJECT;
    case sim_ros2_cancel_response_accept:
        return rclcpp_action::CancelResponse::ACCEPT;
    default:
        sim::addLog(sim_verbosity_scripterrors, "invalid cancel response");
        return rclcpp_action::CancelResponse::REJECT;
    }
}

void ros_action_callback__handle_accepted__`interface.goal.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.goal.cpp_type` *goal, ActionServerProxy *proxy)
{
    int stack = -1;
    try
    {
        stack = sim::createStack();
        write__string(goalUUIDtoString(goal_id), stack, &(proxy->wr_opt));
        write__`interface.goal.cpp_type_normalized`(*goal, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(scriptID, callback, stack);
        sim::releaseStack(stack);
        stack = -1;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_action_callback__handle_accepted__`interface.goal.cpp_type_normalized`: ";
        msg += ex.what();
        simSetLastError(callback, msg.c_str());
    }
}

#py endif
#py endfor
