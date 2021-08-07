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

float b = 0.005;
float x_vel = 0;
float y_vel = 0;

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
      "cmd_vel", 1);

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
          if(msg->transforms[0].child_frame_id !="base_footprint")
           { std::cout << "message not base_footprint" << std::endl;
              return ;
           }
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
            state.x = msg->transforms[0].transform.translation.x + b*cos(yaw);
            state.y = msg->transforms[0].transform.translation.y + b*sin(yaw);

            std::cout << "q0: "   << msg->transforms[0].transform.rotation.x << "q1: "   << msg->transforms[0].transform.rotation.y << "q2: "   << msg->transforms[0].transform.rotation.z << "q3: "   << msg->transforms[0].transform.rotation.w <<  std::endl;
            std::cout << "roll: "   << roll << "pitch : " << pitch  << "yaw  : "  << yaw << std::endl;
            std::cout << "state x: " << msg->transforms[0].transform.translation.x << " state  y: " << msg->transforms[0].transform.translation.y << " state  z: " <<  roll << std::endl;
            

            reference = path.back();

            std::cout << "ref x: " << reference.x << " ref  y: " << reference.y << std::endl;
            
            double dt = 0.05;
            double k1 = 2;
            double k2 = 1;

            double u1_io = k1*(reference.x - state.x) - k2*x_vel;
            double u2_io = k1*(reference.y - state.y) - k2*y_vel;
            RCLCPP_INFO(this->get_logger(), "u1 '%f'", u1_io);
            RCLCPP_INFO(this->get_logger(), "u2 '%f'", u2_io);
            double v = cos(yaw)*u1_io + sin(yaw)*u2_io;
            double w = -sin(yaw)*u1_io/b + cos(yaw)*u2_io/b;

            RCLCPP_INFO(this->get_logger(), "v: '%f'", v);
            RCLCPP_INFO(this->get_logger(), "w: '%f'", w);

            auto commanded_vel = geometry_msgs::msg::Twist();
            commanded_vel.linear.x = v;
            commanded_vel.angular.z = w;
            publisher_command->publish(commanded_vel);
            x_vel = v*cos(yaw);
            y_vel = v*sin(yaw); 

            // check for path empty

            path.pop_back();
            if(path.empty()){
                path_ready = false;
                commanded_vel.linear.x = 0;
                commanded_vel.angular.z = 0;
                publisher_command ->publish(commanded_vel);
                std::cout << "[Debug] : Path end reached" << std::endl;
            }
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
