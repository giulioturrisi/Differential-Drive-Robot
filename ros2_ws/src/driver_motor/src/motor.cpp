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



//TO RUN PIGPIO need sudo 
//sudo su -> source install/setup.bash -> ros2 run driver_motor dc_motor --ros-args -p Kd:=10.0 -p Ki:=1.0 -p W:=1.0

using namespace std::literals;
using std::placeholders::_1;

int tickLeft = 0;
int tickRight = 0;

int tickLeft_forTF = 0;
int tickRight_forTF = 0;
float angle_rad_Left_forTF = 0;
float angle_rad_Right_forTF = 0;

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

float error_vel_integral_Right_outside = 0;
float error_vel_integral_Left_outside = 0;

float Ts_innerLoop = 0.005;
float Ts_innerLoop_nanosecond = Ts_innerLoop*1000000;
float step = 10;

float minimum_pwm = 15;



//3120 tick more or less
static void update_leftEncoder(const int way){
    tickLeft += way;
    tickLeft_forTF += way;
}
static void update_rightEncoder(const int way){
    tickRight += way; 
    tickRight_forTF += way;     
}




class MotorController : public rclcpp::Node{
  public:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_motors_info;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_odom;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_motors_commands;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // gain low level control -----------------------------
    float W = 10;
    float Kd = 30;
    float Ki = 0;
    float Ka = 0;


    MotorController()
    : Node("motors_controller")
    {
        publisher_motors_info = this->create_publisher<std_msgs::msg::Float64MultiArray>("motors_info", 1);
        publisher_odom = this->create_publisher<std_msgs::msg::Float64MultiArray>("odometry", 1);

        subscription_motors_commands = this->create_subscription<geometry_msgs::msg::Twist>(
                    "cmd_vel", 1, std::bind(&MotorController::controller_callback, this, _1));
        timer_ = this->create_wall_timer(5ms,std::bind(&MotorController::timer_callback, this));

        // declare and set external param -------------------------------------
        declare_parameter("W", 10.0);
        declare_parameter("Kd", 30.0); 
        declare_parameter("Ki", 0.0);
        declare_parameter("Ka", 0.0);
        get_parameter("W", W);
        get_parameter("Kd", Kd);
        get_parameter("Ki", Ki);
        get_parameter("Ka", Ka);

        std::cout << "Kd: " << Kd << ", Ki: " << Ki << ", W:" << W << ", Ka: " << Ka << std::endl; 

    }

  private:

    void forward(bool isLeft,double duty_cycle_forw) const{
        //RCLCPP_INFO(this->get_logger(), "duty '%f'", duty_cycle_forw);
        if(isLeft == 1) {
            //left
            gpioPWM(13,(int)duty_cycle_forw);
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

        angle_rad_Left_forTF = (tickLeft_forTF*0.115)*3.14159/180 - angle_rad_Left_forTF;
        angle_rad_Right_forTF = (tickRight_forTF*0.115)*3.14159/180 - angle_rad_Right_forTF;

        delta_s = (r/2.)*(angle_rad_Left_forTF + angle_rad_Right_forTF);
        delta_theta = (r/d)*(angle_rad_Right_forTF - angle_rad_Left_forTF);
        float v_reconstructed = delta_s/0.005;
        float w_reconstructed = delta_theta/0.005;
        auto motors_message = std_msgs::msg::Float64MultiArray();
        motors_message.data.push_back(v_reconstructed);
        motors_message.data.push_back(w_reconstructed);
        motors_message.data.push_back(0.0); //time to complete
        publisher_odom->publish(motors_message);

        tickRight_forTF = 0;
        tickLeft_forTF = 0;


    }
    

    void controller_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{ 

        // calculate velocity of each wheel ----------------------------------------
        float v = msg->linear.x;
        float w = msg->angular.z;
        float vel_d_Left = (2*v - w*d)/(2*r);
        float vel_d_Right = -(2*v + w*d)/(2*r);

     
        
        //LEFT WHEEL variables ------------------------
        double duty_cycle_Left =  0;     
        
        //speed
        float speed_Left = 0;
        float old_speed_Left = speed_Left_outside;
        float speed_filtered_Left = 0;

        //error
        float error_vel_Left = 0;
        float error_vel_integral_Left = error_vel_integral_Left_outside;

        //angle
        float angle_rad_Left = 0;
        float old_angle_rad_Left = 0;


        //RIGHT WHEEL variables ------------------------
        float duty_cycle_Right = 0;

        //speed
        float speed_Right = 0;
        float old_speed_Right = speed_Right_outside;
        float speed_filtered_Right = 0;

        //error
        float error_vel_Right = 0;
        float error_vel_integral_Right = error_vel_integral_Right_outside;
        
        //angle
        float angle_rad_Right = 0;
        float old_angle_rad_Right = 0;


        //resed tick encoders
        tickRight = 0;
        tickLeft = 0;

        auto start_principal = std::chrono::high_resolution_clock::now();
    

        //inner control loop
        for(int i = 0; i < step; i++) {

            auto start = std::chrono::high_resolution_clock::now();

            //LEFT ----------------------------------

            //calculate the new angle and save the last one
            old_angle_rad_Left = angle_rad_Left;    
            angle_rad_Left = (tickLeft*0.115)*3.14159/180;

            //calculate new velocity and filtering
            speed_Left = (angle_rad_Left - old_angle_rad_Left)/Ts_innerLoop;
            speed_filtered_Left = speed_Left*0.7 + (1.0-0.7)*old_speed_Left;

            //save velocity
            old_angle_rad_Left = speed_filtered_Left;
            speed_Left_outside = speed_filtered_Left;


            //calculation errors
            error_vel_Left = vel_d_Left - speed_filtered_Left;
            error_vel_integral_Left += error_vel_Left;
            error_vel_integral_Left_outside = error_vel_integral_Left;
            

            //calculation duty cycle
            duty_cycle_Left = W*(Kd*error_vel_Left + Ki*error_vel_integral_Left);


            //move the motor
            if(vel_d_Left > 0)
                duty_cycle_Left = 15 +  duty_cycle_Left;
            else if(vel_d_Left < 0)
                duty_cycle_Left = -15 +  duty_cycle_Left;
            if(duty_cycle_Left > 255) duty_cycle_Left = 255;
            if(duty_cycle_Left < -255) duty_cycle_Left = -255;

            if(duty_cycle_Left < 0) {
                backward(1,-duty_cycle_Left);
            }
            else {
                forward(1,duty_cycle_Left);
            }


            ////////////////////////////////////////


            //RIGHT ---------------------------------------------

            //calculate the new angle and save the last one
            old_angle_rad_Right = angle_rad_Right;
            angle_rad_Right = (tickRight*0.115)*3.14159/180;

            //calculate new velocity and filtering
            speed_Right = (angle_rad_Right - old_angle_rad_Right)/Ts_innerLoop;
            speed_filtered_Right = speed_Right*0.7 + (1.0 - 0.7)*old_speed_Right; 
        
            //save velocity
            old_speed_Right = speed_filtered_Right;
            speed_Right_outside = speed_filtered_Right;
            
            //calculation errors
            error_vel_Right = vel_d_Right - speed_filtered_Right;
            error_vel_integral_Right += error_vel_Right;
            error_vel_integral_Right_outside = error_vel_integral_Right;


            //calculation duty cycle
            duty_cycle_Right = W*(Kd*error_vel_Right + Ki*error_vel_integral_Right);
            
            
            
            //move the robot
            if(vel_d_Right > 0)
                duty_cycle_Right = minimum_pwm + duty_cycle_Right;
            else if(vel_d_Right < 0)
                duty_cycle_Right = -minimum_pwm + duty_cycle_Right;
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


        auto motors_message = std_msgs::msg::Float64MultiArray();
        motors_message.data.push_back(vel_d_Left);
        motors_message.data.push_back(speed_Left_outside);
        motors_message.data.push_back(vel_d_Right);
        motors_message.data.push_back(speed_Right_outside);
        publisher_motors_info->publish(motors_message);

        auto finish_total = std::chrono::high_resolution_clock::now();
        auto time_total = std::chrono::duration_cast<std::chrono::microseconds>(finish_total - start_principal).count();
        //RCLCPP_INFO(this->get_logger(), "Ts_all_loop '%f'", (float)time_total / 1e6);
        //RCLCPP_INFO(this->get_logger(), "###########");
    }    

    
};

int main(int argc, char * argv[])
{
    std::cout << "########### dc_motor started" << std::endl;
    gpioInitialise();
    // left motor -------------------------------------
    gpioSetMode(20, PI_OUTPUT);
    gpioWrite(20,0);
    gpioSetMode(26, PI_OUTPUT);
    gpioWrite(26,0);


    gpioSetMode(13, PI_OUTPUT);
    gpioSetPWMfrequency(13,50);
    gpioSetPWMrange(13,255);


    // right motor ------------------------------------
    gpioSetMode(5, PI_OUTPUT);
    gpioWrite(5,0);
    gpioSetMode(6, PI_OUTPUT);
    gpioWrite(6,0);

    gpioSetMode(12, PI_OUTPUT);
    gpioSetPWMfrequency(12,50);
    gpioSetPWMrange(12,255);



    re_decoder left(27, 17, update_leftEncoder);
    re_decoder right(22, 23, update_rightEncoder);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

