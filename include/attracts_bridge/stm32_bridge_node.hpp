#ifndef STM32_BRIDGE_NODE_HPP
#define STM32_BRIDGE_NODE_HPP

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class Stm32Bridge : public rclcpp::Node
{
public:
    Stm32Bridge();

private:
    int OpenSerialPort(const std::string& device_name);
    void CmdVelCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;

private:
    std::string device_name_;
    int fd1_;
    int idx_;
};

#endif
