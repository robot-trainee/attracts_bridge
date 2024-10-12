#include "attracts_bridge/stm32_bridge_node.hpp"

Stm32Bridge::Stm32Bridge() : Node("stm32_bridge_node")
{
    // serial_pub_ = this->create_publisher<std_msgs::msg::String>("serial_in", 1000);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Stm32Bridge::CmdVelCB, this, std::placeholders::_1));

    // TODO: 周期決める -> micro_ROSにするから適当でいっか
    // using namespace std::chrono_literals;
    // timer_ = this->create_wall_timer(100ms, std::bind(&Stm32Bridge::TimerCB, this));

    // TODO: device_nameをパラメータに -> micro_ROSにするから適当でいっか
    device_name_ = "/dev/ttyACM0";
    fd1_ = OpenSerialPort(device_name_);
    if (fd1_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
        rclcpp::shutdown();
    }
}

int Stm32Bridge::OpenSerialPort(const std::string& device_name)
{
    int fd1 = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd1, F_SETFL, 0);

    if (fd1_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
        rclcpp::shutdown();
    }

    return fd1;
}

// void Stm32Bridge::TimerCB()
// {
//     // TODO: 受信の仕方考える -> micro_ROSにするから適当でいっか
//     char buf[32] = {0};
//     std::string data;
//     int recv_data = read(fd1_, buf, sizeof(buf));
//     if (recv_data > 0)
//     {
//         RCLCPP_INFO(this->get_logger(), "get!!");

//         // data += std::string(buf, recv_data);

//         // auto serial_msg = std::make_unique<std_msgs::msg::String>();
//         // serial_msg->data = data;
//         // serial_pub_->publish(std::move(serial_msg));
//     }
// }

void Stm32Bridge::CmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    char buf[32]; // メッセージのバッファーサイズを適切に制限
    int bytes_written;

    RCLCPP_INFO(this->get_logger(), "cmd_vel(x: %lf, y: %lf, omega: %lf)", msg->linear.x, msg->linear.y, msg->angular.z);

    // メッセージをバッファに書き込む
    bytes_written = snprintf(buf, sizeof(buf), "%7.5f,%7.5f,%7.5f\r\n", msg->linear.x, msg->linear.y, msg->angular.z);
    RCLCPP_INFO(this->get_logger(), "serial_send: %s", buf);

    if (bytes_written < 0 || bytes_written >= sizeof(buf))
    {
        RCLCPP_ERROR(this->get_logger(), "Serial Failed: message formatting error");
        return;
    }

    int rec = write(fd1_, buf, bytes_written);

    if (rec < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial Failed: could not write");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32Bridge>());
    rclcpp::shutdown();
    return 0;
}
