#include "attracts_bridge/stm32_bridge_node.hpp"

Stm32Bridge::Stm32Bridge() : Node("stm32_bridge_node")
{
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("cmd_vel", 10, std::bind(&Stm32Bridge::CmdVelCB, this, std::placeholders::_1));

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

void Stm32Bridge::CmdVelCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // インデックスを更新
    idx_++;
    if (idx_ >= 6)
    {
        idx_ = 0;
    }
    // idx_ = 0;

    // 元データを確認
    RCLCPP_INFO(
        this->get_logger(), "cmd_vel(0: %lf, 1: %lf, 2: %lf, 3: %lf, 4: %lf, 5: %lf)",
        msg->data.at(0), msg->data.at(1), msg->data.at(2), msg->data.at(3), msg->data.at(4), msg->data.at(5)
    );

    // メッセージのバッファーサイズを適切に制限
    int bytes_written = 8;
    char buf[bytes_written];

    // メッセージをバッファに書き込む
    // ---header
    buf[0] = 0xFF;
    // ---index
    buf[1] = (uint8_t)idx_;
    // ---data
    union {
        float f;
        int32_t ui;
    } data;
    data.f = (float)msg->data.at(idx_);
    buf[2] = (uint8_t)((data.ui & 0xFF000000) >> 24);
    buf[3] = (uint8_t)((data.ui & 0x00FF0000) >> 16);
    buf[4] = (uint8_t)((data.ui & 0x0000FF00) >> 8);
    buf[5] = (uint8_t)((data.ui & 0x000000FF) >> 0);
    // ---end
    buf[6] = 0xFF;

    // check serial data
    RCLCPP_INFO(this->get_logger(), "serial_send:");
    for (int i = 0; i < 8; i++)
    {
        RCLCPP_INFO(this->get_logger(), "%x ", buf[i]);
    }

    // send data
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
