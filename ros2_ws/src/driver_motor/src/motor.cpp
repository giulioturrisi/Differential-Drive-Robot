#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <initializer_list>
#include <iostream>
#include <memory>

#include <pigpio.h>

#include "rotary_encoder.hpp"


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"



//TO RUN PIGPIO need sudo
//sudo su -> source install/setup.bash -> ros2 run driver_motor dc_motor --ros-args -p Kd:=10.0 -p Ki:=1.0 -p W:=1.0

using namespace std::literals;
using std::placeholders::_1;


// Pins -------------------------------------------------------
const int LEFT_PWM = 13;
const int LEFT_IN_1 = 26;
const int LEFT_IN_2 = 20;
const int RIGHT_PWM = 12;
const int RIGHT_IN_1 = 5;
const int RIGHT_IN_2 = 6;

const int LEFT_ENCODER_A = 27;
const int LEFT_ENCODER_B = 17;
const int RIGHT_ENCODER_A = 22;
const int RIGHT_ENCODER_B = 23;

const int PWM_RANGE = 255;
const double RAD_PER_TICK = 0.115*M_PI/180.0;  //3120 tick more or less for each revolution


// Encoder ticks: written by the pigpio thread, read by the control loop --------------
std::atomic<long> tickLeft(0);
std::atomic<long> tickRight(0);

static void update_leftEncoder(const int way){
    tickLeft.fetch_add(way);
}
static void update_rightEncoder(const int way){
    tickRight.fetch_add(way);
}


// Motors ---------------------------------------------------------
static void forward(bool isLeft, double duty_cycle_forw){
    if(isLeft) {
        gpioPWM(LEFT_PWM, (int)duty_cycle_forw);
        gpioWrite(LEFT_IN_1, 0);
        gpioWrite(LEFT_IN_2, 1);
    }
    else {
        gpioPWM(RIGHT_PWM, (int)duty_cycle_forw);
        gpioWrite(RIGHT_IN_1, 1);
        gpioWrite(RIGHT_IN_2, 0);
    }
}

static void backward(bool isLeft, double duty_cycle_back){
    if(isLeft) {
        gpioPWM(LEFT_PWM, (int)duty_cycle_back);
        gpioWrite(LEFT_IN_1, 1);
        gpioWrite(LEFT_IN_2, 0);
    }
    else {
        gpioPWM(RIGHT_PWM, (int)duty_cycle_back);
        gpioWrite(RIGHT_IN_1, 0);
        gpioWrite(RIGHT_IN_2, 1);
    }
}

static void stop_motors(){
    gpioPWM(LEFT_PWM, 0);
    gpioWrite(LEFT_IN_1, 0);
    gpioWrite(LEFT_IN_2, 0);
    gpioPWM(RIGHT_PWM, 0);
    gpioWrite(RIGHT_IN_1, 0);
    gpioWrite(RIGHT_IN_2, 0);
}

// fatal errors: stop the motors and release pigpio (its own signal handler is disabled) before dying
static void fatal_signal_handler(int signal_number){
    stop_motors();
    gpioTerminate();
    std::signal(signal_number, SIG_DFL);
    std::raise(signal_number);
}



class MotorController : public rclcpp::Node{
  public:

    MotorController()
    : Node("motors_controller")
    {
        // declare and set external param -------------------------------------
        W = declare_parameter("W", 1.0);
        Kd = declare_parameter("Kd", 10.0);     // proportional gain on the wheel speed error
        Ki = declare_parameter("Ki", 0.05);
        r = declare_parameter("wheel_radius", 0.05);
        d = declare_parameter("wheel_distance", 0.20);
        minimum_pwm = declare_parameter("minimum_pwm", 15.0);   // static friction compensation
        command_timeout = declare_parameter("command_timeout", 0.5);   // motors stop without commands [s]
        int pwm_frequency = declare_parameter("pwm_frequency", 1000);

        std::cout << "Kd: " << Kd << ", Ki: " << Ki << ", W:" << W << ", r: " << r << ", d: " << d << std::endl;

        // PWM ------------------------------------------------------------------
        for(int pin : {LEFT_PWM, RIGHT_PWM}) {
            int frequency = gpioSetPWMfrequency(pin, pwm_frequency);
            gpioSetPWMrange(pin, PWM_RANGE);
            std::cout << "PWM frequency of gpio " << pin << ": " << frequency << " Hz" << std::endl;
        }

        publisher_motors_info = this->create_publisher<std_msgs::msg::Float64MultiArray>("motors_info", 1);
        publisher_odom = this->create_publisher<geometry_msgs::msg::TwistStamped>("wheel_odometry", 1);

        subscription_motors_commands = this->create_subscription<geometry_msgs::msg::Twist>(
                    "control_vel", 1, std::bind(&MotorController::command_callback, this, _1));

        // low level control and wheel odometry run every Ts_innerLoop, without blocking the executor
        timer_ = this->create_wall_timer(5ms, std::bind(&MotorController::control_callback, this));
    }

    ~MotorController(){
        stop_motors();
    }


  private:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_motors_info;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_odom;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_motors_commands;
    rclcpp::TimerBase::SharedPtr timer_;

    // gain low level control -----------------------------
    double W, Kd, Ki;
    double r, d;
    double minimum_pwm;
    double command_timeout;

    // commands ---------------------------------------------
    double v = 0, w = 0;
    double vel_d_Left = 0, vel_d_Right = 0;
    bool command_received = false;
    std::chrono::steady_clock::time_point last_command_time;

    // wheel state ------------------------------------------
    bool first_iteration = true;
    std::chrono::steady_clock::time_point last_time;
    long last_tick_Left = 0, last_tick_Right = 0;
    double speed_filtered_Left = 0, speed_filtered_Right = 0;
    double error_vel_integral_Left = 0, error_vel_integral_Right = 0;
    double previous_vel_d_Left = 0, previous_vel_d_Right = 0;
    double v_reconstructed = 0, w_reconstructed = 0;


    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        // calculate velocity of each wheel (the right encoder counts backward when the robot goes forward)
        v = msg->linear.x;
        w = msg->angular.z;
        vel_d_Left = (2*v - w*d)/(2*r);
        vel_d_Right = -(2*v + w*d)/(2*r);

        command_received = true;
        last_command_time = std::chrono::steady_clock::now();
    }


    double wheel_controller(double vel_d, double speed, double &error_vel_integral, double &previous_vel_d){
        // the integral of the old direction would push against the new one
        if(vel_d*previous_vel_d < 0) error_vel_integral = 0;
        previous_vel_d = vel_d;

        double error_vel = vel_d - speed;
        double duty_cycle = W*(Kd*error_vel + Ki*(error_vel_integral + error_vel));
        if(vel_d > 0)
            duty_cycle = minimum_pwm + duty_cycle;
        else if(vel_d < 0)
            duty_cycle = -minimum_pwm + duty_cycle;
        double duty_cycle_saturated = std::clamp(duty_cycle, (double)-PWM_RANGE, (double)PWM_RANGE);

        // anti windup: the integral grows only if the motor is not saturated in the direction of the error
        if(duty_cycle == duty_cycle_saturated || (duty_cycle > PWM_RANGE && error_vel < 0) || (duty_cycle < -PWM_RANGE && error_vel > 0))
            error_vel_integral += error_vel;

        return duty_cycle_saturated;
    }


    void control_callback() {
        auto now = std::chrono::steady_clock::now();
        long tick_Left = tickLeft.load();
        long tick_Right = tickRight.load();

        if(first_iteration) {
            first_iteration = false;
            last_time = now;
            last_tick_Left = tick_Left;
            last_tick_Right = tick_Right;
            return;
        }

        // wheel speeds over the real elapsed time -----------------------------
        double dt = std::chrono::duration<double>(now - last_time).count();
        double speed_Left = (tick_Left - last_tick_Left)*RAD_PER_TICK/dt;
        double speed_Right = (tick_Right - last_tick_Right)*RAD_PER_TICK/dt;
        last_time = now;
        last_tick_Left = tick_Left;
        last_tick_Right = tick_Right;

        speed_filtered_Left = speed_Left*0.7 + 0.3*speed_filtered_Left;
        speed_filtered_Right = speed_Right*0.7 + 0.3*speed_filtered_Right;


        // low level control, the motors stop without recent commands ------------
        bool command_valid = command_received &&
            std::chrono::duration<double>(now - last_command_time).count() < command_timeout;
        if(!command_valid || (vel_d_Left == 0 && vel_d_Right == 0)) {
            stop_motors();
            error_vel_integral_Left = 0;
            error_vel_integral_Right = 0;
        }
        else {
            double duty_cycle_Left = wheel_controller(vel_d_Left, speed_filtered_Left, error_vel_integral_Left, previous_vel_d_Left);
            double duty_cycle_Right = wheel_controller(vel_d_Right, speed_filtered_Right, error_vel_integral_Right, previous_vel_d_Right);

            if(duty_cycle_Left < 0) backward(1, -duty_cycle_Left);
            else forward(1, duty_cycle_Left);

            if(duty_cycle_Right < 0) backward(0, -duty_cycle_Right);
            else forward(0, duty_cycle_Right);
        }


        // wheel odometry ------------------------------------------------------
        v_reconstructed = 0.9*(r/2.)*(speed_Left - speed_Right) + 0.1*v_reconstructed;
        w_reconstructed = 0.9*(r/d)*(-speed_Right - speed_Left) + 0.1*w_reconstructed;

        auto odom_message = geometry_msgs::msg::TwistStamped();
        odom_message.twist.linear.x = v_reconstructed;
        odom_message.twist.angular.z = w_reconstructed;
        odom_message.header.stamp = this->get_clock()->now();
        odom_message.header.frame_id = "base_footprint";
        publisher_odom->publish(odom_message);


        // debug info --------------------------------------------------------
        auto motors_message = std_msgs::msg::Float64MultiArray();
        motors_message.data.push_back(vel_d_Left);
        motors_message.data.push_back(speed_filtered_Left);
        motors_message.data.push_back(vel_d_Right);
        motors_message.data.push_back(speed_filtered_Right);

        motors_message.data.push_back(v);
        motors_message.data.push_back((r/2.)*(speed_filtered_Left - speed_filtered_Right));

        motors_message.data.push_back(w);
        motors_message.data.push_back((r/d)*(-speed_filtered_Right - speed_filtered_Left));

        publisher_motors_info->publish(motors_message);
    }
};



int main(int argc, char * argv[])
{
    std::cout << "########### dc_motor started" << std::endl;

    // SIGINT and SIGTERM are left to rclcpp, so that the motors are stopped before exiting
    gpioCfgSetInternals(gpioCfgGetInternals() | PI_CFG_NOSIGHANDLER);
    for(int signal_number : {SIGSEGV, SIGABRT, SIGFPE, SIGBUS})
        std::signal(signal_number, fatal_signal_handler);
    if(gpioInitialise() < 0) {
        std::cerr << "pigpio initialisation failed (the node needs sudo)" << std::endl;
        return 1;
    }

    for(int pin : {LEFT_IN_1, LEFT_IN_2, LEFT_PWM, RIGHT_IN_1, RIGHT_IN_2, RIGHT_PWM})
        gpioSetMode(pin, PI_OUTPUT);
    stop_motors();

    re_decoder left(LEFT_ENCODER_A, LEFT_ENCODER_B, update_leftEncoder);
    re_decoder right(RIGHT_ENCODER_A, RIGHT_ENCODER_B, update_rightEncoder);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());

    stop_motors();
    left.re_cancel();
    right.re_cancel();
    gpioTerminate();
    rclcpp::shutdown();
    return 0;
}
