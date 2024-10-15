#ifndef STM32_BRIDGE_NODE_HPP
#define STM32_BRIDGE_NODE_HPP

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Stm32Bridge : public rclcpp::Node
{
public:
    Stm32Bridge();

private:
    int OpenSerialPort(const std::string& device_name);
    void CmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) const;

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

private:
    std::string device_name_;
    int fd1_;
};

#endif
