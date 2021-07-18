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
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"


//TO RUN sudo -s, then . install/setup.bash 


using std::placeholders::_1;

int tickLeft = 0;
int tickRight = 0;

class MotorController : public rclcpp::Node{
  public:
    

  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_motors_commands;


    MotorController()
    : Node("motors_controller")
    {
        //publisher_motors_commands = this->create_subscription<std_msgs::msg::Float64MultiArray>("motors", 1);

        subscription_motors_commands = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&MotorController::controller_callback, this, _1));

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

    
    void controller_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{

        re_decoder right(22, 23, update_rightEncoder);
        re_decoder left(27, 17, update_leftEncoder);
        //re_decoder right(22, 23, update_rightEncoder);
        
        int W = 1;

        //RCLCPP_INFO(this->get_logger(), "boh '%f'", 0.0);
        
        double duty_cycle_Left =  0;
        double vel_d_Left = msg->linear.x*10;

        //RCLCPP_INFO(this->get_logger(), "VEL DESIRED '%f'", vel_d_Left);


        //RCLCPP_INFO(this->get_logger(), "tickLeft '%i'", tickLeft);
        //RCLCPP_INFO(this->get_logger(), "tickRight '%i'", tickRight);
        
        

        float speed_Left = 0;
        float speed_last_Left = 0;
        float speed_filtered_Left = 0;

        float error_vel_Left = 0;
        float error_acc_Left = 0;
        float error_vel_integral_Left = 0;

        float angle_rad_Left = 0;
        float old_angle_rad_Left = 0;


        float duty_cycle_Right = 0;
        float vel_d_Right = -msg->linear.x*10;

        float speed_Right = 0;
        float speed_last_Right = 0;
        float speed_filtered_Right = 0;

        float error_vel_Right = 0;
        float error_acc_Right = 0;
        float error_vel_integral_Right = 0;
        //RCLCPP_INFO(this->get_logger(), "error_vel_integral_Right beginning '%f'", error_vel_integral_Right);
        

        float angle_rad_Right = 0;
        float old_angle_rad_Right = 0;

        tickRight = 0;
        tickLeft = 0;
        
        //RCLCPP_INFO(this->get_logger(), "vel_d_Right '%f'", vel_d_Right);

        for(int i = 0; i < 50; i++) {

            auto start = std::chrono::high_resolution_clock::now();

            //LEFT
            old_angle_rad_Left = angle_rad_Left;
            angle_rad_Left = (tickLeft*0.20)*3.14159/180;

            //calculation speed
            speed_Left = (angle_rad_Left - old_angle_rad_Left)/0.002;
            speed_filtered_Left = (speed_last_Left + speed_Left)/2;
            speed_last_Left = speed_Left;
            

            //calculation errors
            error_vel_Left = vel_d_Left - speed_filtered_Left;
            error_acc_Left = (speed_last_Left + speed_filtered_Left)/0.002;
            error_vel_integral_Left += error_vel_Left;
            

            //calculation duty cycle
            duty_cycle_Left = W*(30*error_vel_Left + 0.5*error_vel_integral_Left - 0*error_acc_Left);
            RCLCPP_INFO(this->get_logger(), "Desired '%f'", vel_d_Left);
            RCLCPP_INFO(this->get_logger(), "Actual '%f'", speed_filtered_Left);

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
            angle_rad_Right = (tickRight*0.11)*3.14159/180;

            //calculation speed
            speed_Right = (angle_rad_Right - old_angle_rad_Right)/0.002;
            speed_filtered_Right = (speed_last_Right + speed_Right)/2;
            speed_last_Right = speed_Right;
            
            //calculation errors
            error_vel_Right = vel_d_Right - speed_filtered_Right;
            error_acc_Right = (speed_last_Right + speed_filtered_Right)/0.002;
            error_vel_integral_Right += error_vel_Right;

            //RCLCPP_INFO(this->get_logger(), "error_vel_integral_Right '%f'", 0.1*error_vel_integral_Right);

            //RCLCPP_INFO(this->get_logger(), "error_vel_Right '%f'", error_vel_Right);

            //calculation duty cycle
            duty_cycle_Right = W*(30*error_vel_Right + 0.5*error_vel_integral_Right - 2*error_acc_Right*0);
            
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
                
                if(time > 1999)
                    break;
            }
            
            auto finish_total = std::chrono::high_resolution_clock::now();
	        auto time_total = std::chrono::duration_cast<std::chrono::microseconds>(finish_total - start).count();
            Ts_fake = (float)time_total / 1e6;
            //RCLCPP_INFO(this->get_logger(), "time '%f'", time);
            //RCLCPP_INFO(this->get_logger(), "Ts_fake '%f'", Ts_fake);
            //RCLCPP_INFO(this->get_logger(), "Desired '%f'", vel_d_Right);
            //RCLCPP_INFO(this->get_logger(), "Actual '%f'", speed_filtered_Right);
            //RCLCPP_INFO(this->get_logger(), "vel left '%f'", speed_filtered_Left);
            //RCLCPP_INFO(this->get_logger(), "index '%i'", i);

        }


        

        //left.re_cancel();
        //right.re_cancel();
        //forward(0,0);
        //forward(1,0);

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

