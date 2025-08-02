#ifndef TRANSCEIVER_MODULE_BRIDGE_NODE_HPP
#define TRANSCEIVER_MODULE_BRIDGE_NODE_HPP

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>
#include <attracts_msgs/msg/game_data_input.hpp>
#include <attracts_msgs/msg/game_data_robot.hpp>

class TransceiverModuleBridge : public rclcpp::Node
{
public:
    static const int BUFFER_SIZE = 1500;
    static constexpr double CB_RATE = 20.0; // Hz

public:
    TransceiverModuleBridge();

private:
    int OpenSerialPort(const std::string& device_name);
    void TimerCB();
    uint8_t CalculateCRC8(const uint8_t* data, size_t length);
    uint16_t CalculateCRC16(const uint8_t* data, size_t length);
    void ProcessFrame(uint8_t commandType, uint8_t* data, uint16_t dataLength);

private:
    std::string device_name_;
    int fd1_ = -1;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<attracts_msgs::msg::GameDataInput>::SharedPtr game_data_input_pub_;
    rclcpp::Publisher<attracts_msgs::msg::GameDataRobot>::SharedPtr game_data_robot_pub_;

    uint8_t buffer_[BUFFER_SIZE];
    int buffer_index_;
    int control_lost_counter_;

    attracts_msgs::msg::GameDataInput game_data_input_;
    attracts_msgs::msg::GameDataRobot game_data_robot_;
};

#endif
