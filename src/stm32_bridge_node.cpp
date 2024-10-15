#include "attracts_bridge/stm32_bridge_node.hpp"

Stm32Bridge::Stm32Bridge() : Node("stm32_bridge_node")
{
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Stm32Bridge::CmdVelCB, this, std::placeholders::_1));

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
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);

    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    conf_tio.c_lflag &= ~(ECHO | ICANON);

    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    tcsetattr(fd1, TCSANOW, &conf_tio);

    if (fd1_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
        rclcpp::shutdown();
    }

    return fd1;
}

void Stm32Bridge::CmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    char buf[24]; // メッセージのバッファーサイズを適切に制限
    int bytes_written;

    RCLCPP_INFO(this->get_logger(), "cmd_vel(x: %lf, y: %lf, omega: %lf)", msg->linear.x, msg->linear.y, msg->angular.z);

    // メッセージをバッファに書き込む
    bytes_written = snprintf(buf, sizeof(buf), "%5.3f,%5.3f,%5.3f\r\n", msg->linear.x, msg->linear.y, msg->angular.z);
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
