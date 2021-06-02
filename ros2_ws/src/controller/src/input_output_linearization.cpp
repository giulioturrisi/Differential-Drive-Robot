#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nav_msgs/msg/path.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"

using std::placeholders::_1;


struct Point {
    float x;
    float y;
};

std::vector<Point> path;
bool path_ready = false;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("controller")
    {
      subscription_path = this->create_subscription<nav_msgs::msg::Path>(
      "path", 1, std::bind(&MinimalSubscriber::getPath_callback, this, _1));
      
      subscription_state = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 1, std::bind(&MinimalSubscriber::controller_callback, this, _1));
      
      publisher_command = this->create_publisher<geometry_msgs::msg::Twist>(
      "key_vel", 1);

      RCLCPP_INFO(this->get_logger(), "Node started");
      
    }

  private:
    void getPath_callback(const nav_msgs::msg::Path::SharedPtr msg) const
    {
        //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

        float x,y;
        for(int i = 0; i < msg->poses.size() ; i++) {
            x = msg->poses[i].pose.position.x;
            y = msg->poses[i].pose.position.y;
            struct Point new_Point;
            new_Point.x = x;
            new_Point.y = y;
            RCLCPP_INFO(this->get_logger(), "received x'%f'", new_Point.x);
            RCLCPP_INFO(this->get_logger(), "received y'%f'", new_Point.y);
            path.push_back(new_Point);
        }

        path_ready = true;
        RCLCPP_INFO(this->get_logger(), "Path received");
      
    }


    void controller_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
    {
        if(path_ready == true) {
            tf2::Quaternion q(
                msg->transforms[0].transform.rotation.x,
                msg->transforms[0].transform.rotation.y,
                msg->transforms[0].transform.rotation.z,
                msg->transforms[0].transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            struct Point state;
            struct Point reference;
            state.x = msg->transforms[0].transform.translation.x;
            state.y = msg->transforms[0].transform.translation.y;

            RCLCPP_INFO(this->get_logger(), "state x'%f'", msg->transforms[0].transform.translation.x);
            RCLCPP_INFO(this->get_logger(), "state y'%f'", state.y);
            RCLCPP_INFO(this->get_logger(), "state theta'%f'", yaw);
            

            reference = path.back();

            RCLCPP_INFO(this->get_logger(), "reference x'%f'", reference.x);
            RCLCPP_INFO(this->get_logger(), "reference y'%f'", reference.y);
            
            double dt = 0.1;
            double k1 = 1;

            //input-output lin
            //double x_vel = (near_node(1) - path(d - 1,1))/dt;
            //double y_vel = (near_node(2) - path(d - 1,2))/dt;
            double u1_io = k1*(reference.x - state.x);// + x_vel;
            //double u1_io = k1*(1 - state.x);
            double u2_io = k1*(reference.y - state.y);// + y_vel;
            RCLCPP_INFO(this->get_logger(), "u1 '%f'", u1_io);
            RCLCPP_INFO(this->get_logger(), "u2 '%f'", u2_io);
            //double u2_io = k1*(0 - state.y);// + y_vel;
            double v = cos(yaw)*u1_io + sin(yaw)*u2_io;
            double w = -sin(yaw)*u1_io/0.01 + cos(yaw)*u2_io/0.01;

            RCLCPP_INFO(this->get_logger(), "v: '%f'", v);
            RCLCPP_INFO(this->get_logger(), "w: '%f'", w);

            auto commanded_vel = geometry_msgs::msg::Twist();
            commanded_vel.linear.x = v;
            commanded_vel.angular.z = w;

            publisher_command->publish(commanded_vel);



            path.pop_back();
            if(path.empty())
                path_ready = false;
        }
    }


    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_path;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_command;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_state;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}