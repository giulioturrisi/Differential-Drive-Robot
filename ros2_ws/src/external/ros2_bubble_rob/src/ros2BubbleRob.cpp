#include <iostream>
#include <chrono>
#include <boost/lexical_cast.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class BubbleRobNode : public rclcpp::Node
{
private:
    bool sensorTrigger=false;
    unsigned int currentTimeUpdated=0;
    float simulationTime=0.0;
    float driveBackStartTime=-99.0f;
    unsigned int currentTime=0;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSensor;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subSimulationTime;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftMotorSpeedPub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightMotorSpeedPub;
    rclcpp::TimerBase::SharedPtr timer;

public:
    BubbleRobNode(const std::string& leftMotorTopic, const std::string& rightMotorTopic, const std::string& sensorTopic, const std::string& simulationTimeTopic)
        : Node(nodeName())
    {
        currentTimeUpdated=time();
        currentTime=currentTimeUpdated;
        subSensor = this->create_subscription<std_msgs::msg::Bool>(sensorTopic, 1, std::bind(&BubbleRobNode::sensorCallback, this, _1));
        subSimulationTime = this->create_subscription<std_msgs::msg::Float32>(simulationTimeTopic, 1, std::bind(&BubbleRobNode::simulationTimeCallback, this, _1));
        leftMotorSpeedPub = this->create_publisher<std_msgs::msg::Float32>(leftMotorTopic, 1);
        rightMotorSpeedPub = this->create_publisher<std_msgs::msg::Float32>(rightMotorTopic, 1);
        timer = this->create_wall_timer(5ms, std::bind(&BubbleRobNode::timerCallback, this));
    }

    unsigned int time() const
    {
        auto now = std::chrono::steady_clock::now();
        auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
        return now_sec.time_since_epoch().count();
    }

    std::string nodeName()
    {
        currentTimeUpdated=time()&0x00ffffff;
        std::string nodeName("ros2BubbleRob");
        std::string randId(boost::lexical_cast<std::string>(currentTimeUpdated+int(999999.0f*(rand()/(float)RAND_MAX))));
        nodeName+=randId;
        return nodeName;
    }

    // Topic subscription callbacks:

    void sensorCallback(const std_msgs::msg::Bool::SharedPtr sensTrigger)
    {
        currentTimeUpdated=time();
        sensorTrigger=sensTrigger->data;
    }

    void simulationTimeCallback(const std_msgs::msg::Float32::SharedPtr simTime)
    {
        simulationTime=simTime->data;
    }

    void timerCallback()
    {
        // this is the control loop (very simple, just as an example)
        currentTime=time();
        if (currentTime-currentTimeUpdated>9)
            rclcpp::shutdown(); // we didn't receive any sensor information for quite a while... we leave
        float desiredLeftMotorSpeed;
        float desiredRightMotorSpeed;
        if (simulationTime-driveBackStartTime<3.0f)
        { // driving backwards while slightly turning:
            desiredLeftMotorSpeed=-7.0*0.5;
            desiredRightMotorSpeed=-7.0*0.25;
        }
        else
        { // going forward:
            desiredLeftMotorSpeed=7.0;
            desiredRightMotorSpeed=7.0;
            if (sensorTrigger)
                driveBackStartTime=simulationTime; // We detected something, and start the backward mode
            sensorTrigger=false;
        }

        // publish the motor speeds:
        auto d = std_msgs::msg::Float32();
        d.data=desiredLeftMotorSpeed;
        leftMotorSpeedPub->publish(d);
        d.data=desiredRightMotorSpeed;
        rightMotorSpeedPub->publish(d);
    }
};

int main(int argc, char *argv[])
{
    int _argc = 0;
    char** _argv = NULL;
    rclcpp::init(_argc,_argv);

    std::cout << "ros2BubbleRob just started" << std::endl;

    // The robot motor velocities and the sensor topic names are given in the argument list
    // (when CoppeliaSim launches this executable, CoppeliaSim will also provide the argument list)
    std::string leftMotorTopic;
    std::string rightMotorTopic;
    std::string sensorTopic;
    std::string simulationTimeTopic;
    if (argc>=5)
    {
        leftMotorTopic=argv[1];
        rightMotorTopic=argv[2];
        sensorTopic=argv[3];
        simulationTimeTopic=argv[4];
        leftMotorTopic="/"+leftMotorTopic;
        rightMotorTopic="/"+rightMotorTopic;
        sensorTopic="/"+sensorTopic;
        simulationTimeTopic="/"+simulationTimeTopic;
    }
    else
    {
        std::cerr << "Indicate following arguments: 'leftMotorTopic rightMotorTopic sensorTopic simulationTimeTopic'" << std::endl;
        return 1;
    }

    rclcpp::spin(std::make_shared<BubbleRobNode>(leftMotorTopic, rightMotorTopic, sensorTopic, simulationTimeTopic));

    std::cout << "ros2BubbleRob just ended" << std::endl;

    rclcpp::shutdown();

    return 0;
}
