#ifndef SIM_ROS2_PLUGIN__ROS_MSG_BUILTIN_IO__H
#define SIM_ROS2_PLUGIN__ROS_MSG_BUILTIN_IO__H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

struct ROS2ReadOptions
{
    bool uint8array_as_string;
    ROS2ReadOptions();
};

struct ROS2WriteOptions
{
    bool uint8array_as_string;
    ROS2WriteOptions();
};

void read__bool(int stack, bool *value, const ROS2ReadOptions *opt = NULL);
void read__byte(int stack, uint8_t *value, const ROS2ReadOptions *opt = NULL);
void read__char(int stack, unsigned char *value, const ROS2ReadOptions *opt = NULL);
void read__char(int stack, signed char *value, const ROS2ReadOptions *opt = NULL);
void read__int8(int stack, int8_t *value, const ROS2ReadOptions *opt = NULL);
void read__uint8(int stack, uint8_t *value, const ROS2ReadOptions *opt = NULL);
void read__int16(int stack, int16_t *value, const ROS2ReadOptions *opt = NULL);
void read__uint16(int stack, uint16_t *value, const ROS2ReadOptions *opt = NULL);
void read__int32(int stack, int32_t *value, const ROS2ReadOptions *opt = NULL);
void read__uint32(int stack, uint32_t *value, const ROS2ReadOptions *opt = NULL);
void read__int64(int stack, int64_t *value, const ROS2ReadOptions *opt = NULL);
void read__uint64(int stack, uint64_t *value, const ROS2ReadOptions *opt = NULL);
void read__float32(int stack, float *value, const ROS2ReadOptions *opt = NULL);
void read__float64(int stack, double *value, const ROS2ReadOptions *opt = NULL);
void read__string(int stack, std::string *value, const ROS2ReadOptions *opt = NULL);
void read__time(int stack, rclcpp::Time *value, const ROS2ReadOptions *opt = NULL);
void read__duration(int stack, rclcpp::Duration *value, const ROS2ReadOptions *opt = NULL);
void write__bool(bool value, int stack, const ROS2WriteOptions *opt = NULL);
void write__byte(uint8_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__char(unsigned char value, int stack, const ROS2WriteOptions *opt = NULL);
void write__char(signed char value, int stack, const ROS2WriteOptions *opt = NULL);
void write__int8(int8_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__uint8(uint8_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__int16(int16_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__uint16(uint16_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__int32(int32_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__uint32(uint32_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__int64(int64_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__uint64(uint64_t value, int stack, const ROS2WriteOptions *opt = NULL);
void write__float32(float value, int stack, const ROS2WriteOptions *opt = NULL);
void write__float64(double value, int stack, const ROS2WriteOptions *opt = NULL);
void write__string(std::string value, int stack, const ROS2WriteOptions *opt = NULL);
void write__time(rclcpp::Time value, int stack, const ROS2WriteOptions *opt = NULL);
void write__duration(rclcpp::Duration value, int stack, const ROS2WriteOptions *opt = NULL);
std::string goalUUIDtoString(const rclcpp_action::GoalUUID &uuid);
rclcpp_action::GoalUUID goalUUIDfromString(const std::string &uuidStr);

#endif // SIM_ROS2_PLUGIN__ROS_MSG_BUILTIN_IO__H
