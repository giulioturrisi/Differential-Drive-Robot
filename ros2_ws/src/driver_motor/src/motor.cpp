#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <pigpio.h>

#include "rotary_encoder.hpp"
#include <chrono>
#include <unistd.h>
#include <cmath>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"



//TO RUN sudo -s, then . install/setup.bash 

using namespace std::literals;
using std::placeholders::_1;

int tickLeft = 0;
int tickRight = 0;
float odom_x = 0;
float odom_y = 0;
float odom_theta = 0;
float angleRight_sum = 0;
float angleLeft_sum = 0;
float delta_s = 0;
float delta_theta = 0;

float Ts = 0.01;
float r = 0.45;
float d = 0.2;

float speed_Right_outside = 0;
float speed_Left_outside = 0;


float Ts_innerLoop = 0.001;
float Ts_innerLoop_nanosecond = Ts_innerLoop*1000000;
float step = 100;

//float error_vel_integral_Left = 0;
//float error_vel_integral_Right = 0;

class MotorController : public rclcpp::Node{
  public:
    

  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_motors_commands;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    


    MotorController()
    : Node("motors_controller")
    {
        //publisher_motors_commands = this->create_subscription<std_msgs::msg::Float64MultiArray>("motors", 1);

        subscription_motors_commands = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&MotorController::controller_callback, this, _1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(10ms,std::bind(&MotorController::timer_callback, this));

        gpioInitialise();
        //left
        gpioSetMode(20, PI_OUTPUT);
        gpioWrite(20,0);
        gpioSetMode(26, PI_OUTPUT);
        gpioWrite(26,0);


        gpioSetMode(13, PI_OUTPUT);
        gpioSetPWMfrequency(13,50);
        gpioSetPWMrange(13,255);


        gpioSetMode(5, PI_OUTPUT);
        gpioWrite(5,0);
        gpioSetMode(6, PI_OUTPUT);
        gpioWrite(6,0);

        gpioSetMode(12, PI_OUTPUT);
        gpioSetPWMfrequency(12,50);
        gpioSetPWMrange(12,255);
        
        //re_decoder left(27, 17, update_leftEncoder);

        //re_decoder right(22, 23, update_rightEncoder);
    }

  private:

    static void update_leftEncoder(const int way){
        tickLeft += way;
    }
    static void update_rightEncoder(const int way){
        tickRight += way;       
    }


    void forward(bool isLeft,double duty_cycle_forw) const{
        //RCLCPP_INFO(this->get_logger(), "duty '%f'", duty_cycle_forw);
        if(isLeft == 1) {
            //left
            gpioPWM(13,(int)duty_cycle_forw);
            //gpioWrite(26,1);
            //gpioWrite(20,0);
            gpioWrite(26,0);
            gpioWrite(20,1);
        }
        else{
            //right
            gpioPWM(12,(int)duty_cycle_forw);
            gpioWrite(5,1);
            gpioWrite(6,0);

        }
    }


    void backward(bool isLeft,double duty_cycle_back) const{
        //RCLCPP_INFO(this->get_logger(), "duty '%f'", duty_cycle_back);
      
        if(isLeft == 1) {
            //left
            gpioPWM(13,(int)duty_cycle_back);
            //gpioWrite(26,0);
            //gpioWrite(20,1);
            gpioWrite(26,1);
            gpioWrite(20,0);
        }
        else{
            //right
            gpioPWM(12,(int)duty_cycle_back);
            gpioWrite(5,0);
            gpioWrite(6,1);
        }
    }

    
    void timer_callback() {
        

        // euler to quat
        float roll = 0;
        float pitch = 0;
        float yaw = odom_theta;
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        //Quaternion q;
        //q.w = cr * cp * cy + sr * sp * sy;
        //q.x = sr * cp * cy - cr * sp * sy;
        //q.y = cr * sp * cy + sr * cp * sy;
        //q.z = cr * cp * sy - sr * sp * cy;



        //i should publish something
        geometry_msgs::msg::TransformStamped transform_stamped; 
        transform_stamped.header.stamp = rclcpp::Time();
        transform_stamped.header.frame_id = "odom";
        //transform_stamped.child_frame_id = "base_link";
        transform_stamped.child_frame_id = "base_footprint";
        transform_stamped.transform.translation.x = odom_x;
        transform_stamped.transform.translation.y = odom_y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.w = cr * cp * cy + sr * sp * sy;
        transform_stamped.transform.rotation.x = sr * cp * cy - cr * sp * sy;
        transform_stamped.transform.rotation.y = cr * sp * cy + sr * cp * sy;
        transform_stamped.transform.rotation.z = cr * cp * sy - sr * sp * cy;
         
        tf_broadcaster_->sendTransform(transform_stamped);

        //std::cout << "x: " << odom_x << " y: " << odom_y << " theta: " << odom_theta << std::endl;

      

    }
    
    void controller_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{

        re_decoder right(22, 23, update_rightEncoder);
        re_decoder left(27, 17, update_leftEncoder);
        
        int W = 10;

        //calculate velocity of each wheel
        float v = msg->linear.x*2;
        float w = msg->angular.z*2;

        

        double vel_d_Left = (2*v - w*d)/(2*r);
        double vel_d_Right = -(2*v + w*d)/(2*r);

        //std::cout << "velocity linear " << v << " angular " << w << std::endl;
        std::cout << "vel_d_Left " << vel_d_Left << " vel_d_Right " << vel_d_Right << std::endl;
        //LEFT WHEEL
        double duty_cycle_Left =  0;     


        float speed_Left = 0;
        //float speed_last_Left = 0;
        float speed_last_Left = speed_Left_outside;
        float speed_filtered_Left = 0;

        float error_vel_Left = 0;
        float error_acc_Left = 0;
        float error_vel_integral_Left = 0;

        float angle_rad_Left = 0;
        float old_angle_rad_Left = 0;

        //RIGHT WHEEL
        float duty_cycle_Right = 0;

        float speed_Right = 0;
        //float speed_last_Right = 0;
        float speed_last_Right = speed_Right_outside;
        float speed_filtered_Right = 0;

        float error_vel_Right = 0;
        float error_acc_Right = 0;
        float error_vel_integral_Right = 0;
        

        float angle_rad_Right = 0;
        float old_angle_rad_Right = 0;



        tickRight = 0;
        tickLeft = 0;
        speed_Right_outside = 0;
        speed_Left_outside = 0;
        
        
        //inner control loop
        for(int i = 0; i < step; i++) {

            auto start = std::chrono::high_resolution_clock::now();

            //LEFT
            old_angle_rad_Left = angle_rad_Left;
            angle_rad_Left = (tickLeft*0.20)*3.14159/180;

            //calculation speed
            speed_Left = (angle_rad_Left - old_angle_rad_Left)/Ts_innerLoop;
            speed_filtered_Left = (speed_last_Left + speed_Left)/2;
            speed_last_Left = speed_Left;
            speed_Left_outside = speed_Left_outside + speed_Left;
            

            //calculation errors
            error_vel_Left = vel_d_Left - speed_filtered_Left;
            error_acc_Left = (speed_last_Left + speed_filtered_Left)/Ts_innerLoop;
            error_vel_integral_Left += error_vel_Left;
            

            //calculation duty cycle
            duty_cycle_Left = W*(30*error_vel_Left + 2.5*error_vel_integral_Left - 0.*error_acc_Left);


            //move
            if(vel_d_Left > 0)
                duty_cycle_Left = 5*0 +  duty_cycle_Left;
            else
                duty_cycle_Left = -5*0 +  duty_cycle_Left;
            if(duty_cycle_Left > 255) duty_cycle_Left = 255;
            if(duty_cycle_Left < -255) duty_cycle_Left = -255;
            if(duty_cycle_Left < 0) {
                backward(1,-duty_cycle_Left);
            }
            else {
                forward(1,duty_cycle_Left);
            }



            ////////////////////////////////////////


            //RIGHT
            old_angle_rad_Right = angle_rad_Right;
            //angle_rad_Right = (tickRight*0.11)*3.14159/180;
            angle_rad_Right = (tickRight*0.20)*3.14159/180;

            //calculation speed
            speed_Right = (angle_rad_Right - old_angle_rad_Right)/Ts_innerLoop;
            speed_filtered_Right = (speed_last_Right + speed_Right)/2;
            speed_last_Right = speed_Right;
            speed_Right_outside = speed_Right_outside + speed_Right;
            
            //calculation errors
            error_vel_Right = vel_d_Right - speed_filtered_Right;
            error_acc_Right = (speed_last_Right + speed_filtered_Right)/Ts_innerLoop;
            error_vel_integral_Right += error_vel_Right;

            //calculation duty cycle
            duty_cycle_Right = W*(30*error_vel_Right + 2.5*error_vel_integral_Right - 0.*error_acc_Right);
            
            
            
            //move
            if(vel_d_Right > 0)
                duty_cycle_Right = 5*0 + duty_cycle_Right;
            else
                duty_cycle_Right = -5*0 + duty_cycle_Right;
            if(duty_cycle_Right > 255) duty_cycle_Right = 255;
            if(duty_cycle_Right < -255) duty_cycle_Right = -255;
            if(duty_cycle_Right < 0) {
                backward(0,-duty_cycle_Right);
            }
            else {
                forward(0,duty_cycle_Right);
            }

            
            //SLEEP
            float Ts_fake = 0;
            while (true){
                auto finish = std::chrono::high_resolution_clock::now();
                auto time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
                
                if(time > Ts_innerLoop_nanosecond)
                    break;
            }
            
            auto finish_total = std::chrono::high_resolution_clock::now();
	        auto time_total = std::chrono::duration_cast<std::chrono::microseconds>(finish_total - start).count();
            Ts_fake = (float)time_total / 1e6;
            //RCLCPP_INFO(this->get_logger(), "Ts_fake '%f'", Ts_fake);


        }


        

        //left.re_cancel();
        //right.re_cancel();
        
        //maybe to eliminate
        forward(0,0);
        forward(1,0);

        angleRight_sum = angle_rad_Right + angleRight_sum;
        angleLeft_sum = angle_rad_Left + angleLeft_sum;
        
        delta_s = (r/2.)*(angleRight_sum + angleLeft_sum) - delta_s;
        delta_theta = (r/d)*(angleRight_sum - angleLeft_sum) - delta_theta;
        float v_reconstructed = delta_s/Ts;
        float w_reconstructed = delta_theta/Ts;

        odom_x = odom_x + v_reconstructed*Ts*cos(odom_theta);
        odom_y = odom_y + v_reconstructed*Ts*sin(odom_theta);
        odom_theta = odom_theta + w_reconstructed*Ts;
        
        std::cout << "speed left: " << speed_Left_outside/step << " speed Right: " << speed_Right_outside/step << std::endl;
        std::cout << "x: " << odom_x << " y: " << odom_y << " theta: " << odom_theta << std::endl;
        //speed_Left_outside = 0;
        //speed_Right_outside = 0;

        RCLCPP_INFO(this->get_logger(), "###########");
    }    

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}

