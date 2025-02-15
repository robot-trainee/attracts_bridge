#ifndef STM32_BRIDGE_NODE_HPP
#define STM32_BRIDGE_NODE_HPP

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <attracts_msgs/msg/attracts_command.hpp>

class Stm32Bridge : public rclcpp::Node
{
public:
    Stm32Bridge();

private:
    int OpenSerialPort(const std::string& device_name);
    void CmdCB(const attracts_msgs::msg::AttractsCommand::SharedPtr msg);
    void SendSerialData(const uint8_t buf[8]);

private:
    rclcpp::Subscription<attracts_msgs::msg::AttractsCommand>::SharedPtr cmd_sub_;

private:
    std::string device_name_;
    int fd1_ = -1;
};

#endif
