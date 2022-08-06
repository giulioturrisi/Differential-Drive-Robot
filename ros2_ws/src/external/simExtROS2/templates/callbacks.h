#ifndef SIM_ROS2_PLUGIN__CALLBACKS__H
#define SIM_ROS2_PLUGIN__CALLBACKS__H

#include <ros_msg_builtin_io.h>
#include <sim_ros2_interface.h>

#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#include <`interface.cpp_include`>
#py endfor

#py for interface_name, interface in interfaces.items():
#py for subinterface_name, subinterface in interface.subinterfaces.items():
void write__`subinterface.cpp_type_normalized`(const `subinterface.cpp_type`& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__`subinterface.cpp_type_normalized`(int stack, `subinterface.cpp_type` *msg, const ROS2ReadOptions *opt = NULL);
#py endfor
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
void ros_callback__`interface.cpp_type_normalized`(const `interface.cpp_type`::SharedPtr msg, SubscriptionProxy *proxy);
#py endif
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'srv':
bool ros_srv_callback__`interface.cpp_type_normalized`(const std::shared_ptr<rmw_request_id_t> request_header, const `interface.request.cpp_type`::SharedPtr req, `interface.response.cpp_type`::SharedPtr res, ServiceProxy *proxy);
#py endif
#py endfor
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
void ros_action_callback__`interface.feedback.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.feedback.cpp_type` *feedback, ActionClientProxy *proxy);
void ros_action_callback__`interface.result.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, int action_result_code, const `interface.result.cpp_type`::SharedPtr result, ActionClientProxy *proxy);
rclcpp_action::GoalResponse ros_action_callback__handle_goal__`interface.goal.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.goal.cpp_type` *goal, ActionServerProxy *proxy);
rclcpp_action::CancelResponse ros_action_callback__handle_cancel__`interface.goal.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.goal.cpp_type` *goal, ActionServerProxy *proxy);
void ros_action_callback__handle_accepted__`interface.goal.cpp_type_normalized`(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const `interface.goal.cpp_type` *goal, ActionServerProxy *proxy);
#py endif
#py endfor

#endif // SIM_ROS2_PLUGIN__CALLBACKS__H
