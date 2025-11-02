#include "attracts_bridge/transceiver_module_bridge_node.hpp"

TransceiverModuleBridge::TransceiverModuleBridge()
: Node("transceiver_module_bridge_node"), buffer_index_(0), control_lost_counter_(0)
{
    device_name_ = "/dev/ttyUSB0";
    fd1_ = OpenSerialPort(device_name_);
    if (fd1_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
        rclcpp::shutdown();
    }

    game_data_input_pub_ =
        this->create_publisher<attracts_msgs::msg::GameDataInput>("game_data_input", 10);
    game_data_robot_pub_ =
        this->create_publisher<attracts_msgs::msg::GameDataRobot>("game_data_robot", 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / CB_RATE), std::bind(&TransceiverModuleBridge::TimerCB, this));
}

int TransceiverModuleBridge::OpenSerialPort(const std::string& device_name)
{
    int fd1 = open(device_name.c_str(), O_RDWR | O_NOCTTY);

    fcntl(fd1, F_SETFL, FNDELAY);
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);

    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    conf_tio.c_cflag &= ~PARENB;        // パリティなし
    conf_tio.c_cflag &= ~CSTOPB;        // ストップビット1
    conf_tio.c_cflag &= ~CSIZE;
    conf_tio.c_cflag |= CS8;            // データビット8
    conf_tio.c_cflag |= CLOCAL | CREAD; // ローカル接続・受信有効
    conf_tio.c_cflag &= ~CRTSCTS;       // ハードウェアフロー制御なし

    conf_tio.c_iflag = IGNPAR;          // パリティエラー無視

    conf_tio.c_lflag &= ~(ECHO | ICANON);

    tcflush(fd1, TCIOFLUSH);
    tcsetattr(fd1, TCSANOW, &conf_tio);

    return fd1;
}

void TransceiverModuleBridge::TimerCB()
{
    // タイムアウト処理
    ++control_lost_counter_;
    if (control_lost_counter_ > 1000) {
        // 全入力をリセット
        game_data_input_ = attracts_msgs::msg::GameDataInput();
    }

    // 情報を送信
    game_data_input_pub_->publish(game_data_input_);
    game_data_robot_pub_->publish(game_data_robot_);

    // シリアルデータ受信
    buffer_index_ = read(fd1_, buffer_, sizeof(buffer_) - 1);

    // フレーム処理
    if (buffer_index_ >= 7) {
        uint8_t startOfFrame = buffer_[0];
        uint8_t commandType = buffer_[1];
        uint16_t dataLength = buffer_[2] + (buffer_[3] << 8);  // リトルエンディアン
        uint8_t crc8 = buffer_[4];

        // ヘッダー検証
        if (startOfFrame != 0xAE || dataLength > 993 || CalculateCRC8(buffer_, 4) != crc8) {
            // エラー: 1バイト進める
            for (int i = 0; i < buffer_index_ - 1; ++i) {
                buffer_[i] = buffer_[i + 1];
            }
            --buffer_index_;
        }
        else if (buffer_index_ >= dataLength + 7) {
            // CRC-16検証
            uint16_t crc16 = buffer_[buffer_index_ - 2] + (buffer_[buffer_index_ - 1] << 8);
            if (CalculateCRC16(buffer_, buffer_index_ - 2) != crc16) {
                // CRCエラー: 1バイト進める
                for (int i = 0; i < buffer_index_ - 1; ++i) {
                    buffer_[i] = buffer_[i + 1];
                }
                --buffer_index_;
            }
            else {
                // 正常フレーム処理
                ProcessFrame(commandType, buffer_ + 5, dataLength);

                // バッファを更新
                int frameSize = dataLength + 7;
                for (int i = 0; i < buffer_index_ - frameSize; ++i) {
                    buffer_[i] = buffer_[i + frameSize];
                }
                buffer_index_ -= frameSize;
            }
        }
    }
}

uint8_t TransceiverModuleBridge::CalculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t TransceiverModuleBridge::CalculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void TransceiverModuleBridge::ProcessFrame(uint8_t commandType, uint8_t* data, uint16_t dataLength) {
    if (commandType == 0x00 && dataLength == 5) {
        // 入力データ処理
        struct DataInput {
            int16_t mouseDeltaX;
            int16_t mouseDeltaY;
            uint8_t buttons;
        } __attribute__((__packed__));

        DataInput* input = (DataInput*)data;
        game_data_input_.mouse_delta_x = input->mouseDeltaX;
        game_data_input_.mouse_delta_y = input->mouseDeltaY;
        game_data_input_.mouse_left_button = input->buttons & 0x01;
        game_data_input_.mouse_right_button = input->buttons & 0x02;
        game_data_input_.key_w = input->buttons & 0x04;
        game_data_input_.key_a = input->buttons & 0x08;
        game_data_input_.key_s = input->buttons & 0x10;
        game_data_input_.key_d = input->buttons & 0x20;

        control_lost_counter_ = 0;  // 通信正常
    }
    else if (commandType == 0x01 && dataLength == 11) {
        // ロボット情報処理
        struct DataRobot {
            uint8_t type;
            uint8_t team;
            uint8_t projectileSpeedMax;
            uint16_t maxHP;
            uint16_t currentHP;
            uint16_t maxHeat;
            uint16_t currentHeat;
        } __attribute__((__packed__));

        DataRobot* robot = (DataRobot*)data;
        game_data_robot_.type = robot->type;
        game_data_robot_.team = robot->team;
        game_data_robot_.projectile_speed_max = robot->projectileSpeedMax;
        game_data_robot_.max_hp = robot->maxHP;
        game_data_robot_.current_hp = robot->currentHP;
        game_data_robot_.max_heat = robot->maxHeat;
        game_data_robot_.current_heat = robot->currentHeat;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransceiverModuleBridge>());
    rclcpp::shutdown();
    return 0;
}
