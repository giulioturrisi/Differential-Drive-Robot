#include <ros_msg_builtin_io.h>
#include <simLib.h>
#include <iostream>
#include <stubs.h>

ROS2ReadOptions::ROS2ReadOptions()
    : uint8array_as_string(false)
{
}

ROS2WriteOptions::ROS2WriteOptions()
    : uint8array_as_string(false)
{
}

void read__bool(int stack, bool *value, const ROS2ReadOptions *opt)
{
    simBool v;
    if(sim::getStackBoolValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected bool");
    }
}

void read__byte(int stack, uint8_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = (uint8_t)v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected bool");
    }
}

void read__char(int stack, unsigned char *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = (char)v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected bool");
    }
}

void read__char(int stack, signed char *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = (signed char)v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected bool");
    }
}

void read__int8(int stack, int8_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__uint8(int stack, uint8_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__int16(int stack, int16_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__uint16(int stack, uint16_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__int32(int stack, int32_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__uint32(int stack, uint32_t *value, const ROS2ReadOptions *opt)
{
    simInt v;
    if(sim::getStackInt32Value(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected integer");
    }
}

void read__int64(int stack, int64_t *value, const ROS2ReadOptions *opt)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    simDouble v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__uint64(int stack, uint64_t *value, const ROS2ReadOptions *opt)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    simDouble v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__float32(int stack, float *value, const ROS2ReadOptions *opt)
{
    simFloat v;
    if(sim::getStackFloatValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected float");
    }
}

void read__float64(int stack, double *value, const ROS2ReadOptions *opt)
{
    simDouble v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = v;
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__string(int stack, std::string *value, const ROS2ReadOptions *opt)
{
    simChar *str;
    simInt strSize;
    if((str = sim::getStackStringValue(stack, &strSize)) != NULL && strSize > 0)
    {
        *value = std::string(str, strSize);
        sim::popStackItem(stack, 1);
        sim::releaseBuffer(str);
    }
    else
    {
        throw sim::exception("expected string");
    }
}

void read__time(int stack, rclcpp::Time *value, const ROS2ReadOptions *opt)
{
    simDouble v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = rclcpp::Time(v);
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void read__duration(int stack, rclcpp::Duration *value, const ROS2ReadOptions *opt)
{
    simDouble v;
    if(sim::getStackDoubleValue(stack, &v) == 1)
    {
        *value = rclcpp::Duration::from_seconds(v);
        sim::popStackItem(stack, 1);
    }
    else
    {
        throw sim::exception("expected double");
    }
}

void write__bool(bool value, int stack, const ROS2WriteOptions *opt)
{
    simBool v = value;
    sim::pushBoolOntoStack(stack, v);
}

void write__byte(uint8_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__char(unsigned char value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__char(signed char value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int8(int8_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__uint8(uint8_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int16(int16_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__uint16(uint16_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int32(int32_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__uint32(uint32_t value, int stack, const ROS2WriteOptions *opt)
{
    simInt v = value;
    sim::pushInt32OntoStack(stack, v);
}

void write__int64(int64_t value, int stack, const ROS2WriteOptions *opt)
{
    // XXX: we represent Int64 as double - possible loss of precision!
    simDouble v = value;
    sim::pushDoubleOntoStack(stack, v);
}

void write__uint64(uint64_t value, int stack, const ROS2WriteOptions *opt)
{
    // XXX: we represent UInt64 as double - possible loss of precision!
    simDouble v = value;
    sim::pushDoubleOntoStack(stack, v);
}

void write__float32(float value, int stack, const ROS2WriteOptions *opt)
{
    simFloat v = value;
    sim::pushFloatOntoStack(stack, v);
}

void write__float64(double value, int stack, const ROS2WriteOptions *opt)
{
    simDouble v = value;
    sim::pushDoubleOntoStack(stack, v);
}

void write__string(std::string value, int stack, const ROS2WriteOptions *opt)
{
    const simChar *v = value.c_str();
    sim::pushStringOntoStack(stack, v, value.length());
}

void write__time(rclcpp::Time value, int stack, const ROS2WriteOptions *opt)
{
    simDouble v = value.seconds();
    sim::pushDoubleOntoStack(stack, v);
}

void write__duration(rclcpp::Duration value, int stack, const ROS2WriteOptions *opt)
{
    simDouble v = value.seconds();
    sim::pushDoubleOntoStack(stack, v);
}

std::string goalUUIDtoString(const rclcpp_action::GoalUUID &uuid)
{
    static char n[17] = "0123456789abcdef";
    std::stringstream ss;
    for(size_t i = 0; i < UUID_SIZE; i++)
    {
        int h = (uuid[i] >> 4) & 0x0F;
        int l = (uuid[i] >> 0) & 0x0F;
        ss << n[h] << n[l];
    }
    return ss.str();
}

rclcpp_action::GoalUUID goalUUIDfromString(const std::string &uuidStr)
{
    static int val[256] = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,
        0,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,10,11,12,13,14,15,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };

    if(uuidStr.size() != UUID_SIZE * 2)
        sim::addLog(sim_verbosity_warnings, "uuid '%s' has not the correct length (%d bytes)", uuidStr, UUID_SIZE);

    rclcpp_action::GoalUUID ret;
    for(size_t i = 0, j = 0; i < (uuidStr.size() & ~1); i += 2, j++)
        ret[j] = (val[uuidStr[i]] << 4) | val[uuidStr[i+1]];
    return ret;
}

