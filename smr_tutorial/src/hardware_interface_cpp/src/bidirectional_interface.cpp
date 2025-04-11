#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#define HEAD 0xFF
#define DEVICE_ID 0x01
#define FUNC_RANGE 0x03
#define FUNC_MOTION 0x02
#define SERIALPORT_TIMEOUT_MS 100

class BidirectionalInterface : public rclcpp::Node
{
public:
    BidirectionalInterface()
    : Node("bidirectional_interface"), run_receive_thread_(true)
    {
        // Setup serial connection
        try {
            serial_.Open("/dev/esp32"); // Open the serial port
            serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); // Set baud rate
            serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8); // Set character size
            serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1); // Set stop bits
            serial_.SetParity(LibSerial::Parity::PARITY_NONE); // Set parity
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        } catch (const LibSerial::OpenFailed&) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            rclcpp::shutdown();
            return;
        }

        // Create publisher for ultrasonic sensor data
        pub_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic_distance", 10);

        // Subscribe to /cmd_vel topic
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&BidirectionalInterface::cmdVelCallback, this, std::placeholders::_1));

        // Start a thread to receive data
        receive_thread_ = std::thread(&BidirectionalInterface::receiveData, this);
    }

    ~BidirectionalInterface()
    {
        run_receive_thread_ = false;

        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }

        if (serial_.IsOpen()) {
            serial_.Close(); // Close the serial port
        }
    }

private:
    void receiveData()
    {
        try {
            uint8_t header;
            uint8_t device_id;
            uint8_t len;
            uint8_t func_type;
            uint8_t data_len;
            uint8_t value;
            uint8_t rx_check_num;
            uint8_t check_sum;
            std::vector<uint8_t> data;

            while (run_receive_thread_) {

                if (serial_.IsDataAvailable()) {
                    serial_.ReadByte(header, SERIALPORT_TIMEOUT_MS);

                    if (header == HEAD) {
                        serial_.ReadByte(device_id, SERIALPORT_TIMEOUT_MS);

                        if (device_id == DEVICE_ID) {
                            serial_.ReadByte(len, SERIALPORT_TIMEOUT_MS);
                            serial_.ReadByte(func_type, SERIALPORT_TIMEOUT_MS);

                            check_sum = header + device_id + len + func_type;
                            data_len = len - 4;
                            data.clear();

                            while (data.size() < data_len) {
                                serial_.ReadByte(value, SERIALPORT_TIMEOUT_MS);
                                data.push_back(value);
                                check_sum += value;
                            }

                            serial_.ReadByte(rx_check_num, SERIALPORT_TIMEOUT_MS);

                            if ((check_sum & 0xFF) == rx_check_num) {
                                if (func_type == FUNC_RANGE) {
                                    parseRangeData(data);
                                }
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Checksum error in received data.");
                            }
                        }
                    }
                } else {
                    usleep(10000); // Sleep for 10ms
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in receiveData: %s", e.what());
        }
    }

    void parseRangeData(const std::vector<uint8_t>& data)
    {
        if (data.size() >= 2) {
            int16_t range = static_cast<int16_t>(data[0] | (data[1] << 8));
            float distance = range / 1000.0; // Convert to meters

            auto msg = std_msgs::msg::Float32();
            msg.data = distance;
            pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published distance: %.2f meters", distance);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid range data received.");
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (serial_.IsOpen()) {
            try {
                // Convert linear and angular velocities to integers (scaled by 1000)
                int16_t v_x = static_cast<int16_t>(msg->linear.x * 1000);
                int16_t v_y = static_cast<int16_t>(msg->linear.y * 1000);
                int16_t v_z = static_cast<int16_t>(msg->angular.z * 1000);

                // Prepare the command packet
                std::vector<uint8_t> cmd = {
                    HEAD, DEVICE_ID, 0x00, FUNC_MOTION,
                    static_cast<uint8_t>(v_x & 0xFF), static_cast<uint8_t>((v_x >> 8) & 0xFF),
                    static_cast<uint8_t>(v_y & 0xFF), static_cast<uint8_t>((v_y >> 8) & 0xFF),
                    static_cast<uint8_t>(v_z & 0xFF), static_cast<uint8_t>((v_z >> 8) & 0xFF)
                };

                // Set the length of the packet
                cmd[2] = cmd.size();

                // Calculate checksum
                uint8_t checksum = 0;
                for (const auto& byte : cmd) {
                    checksum += byte;
                }
                checksum &= 0xFF;
                cmd.push_back(checksum);

                // Send the command packet
                serial_.Write(cmd);

                RCLCPP_INFO(this->get_logger(), "Sent /cmd_vel to controller: v_x=%.2f, v_y=%.2f, v_z=%.2f",
                            msg->linear.x, msg->linear.y, msg->angular.z);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error sending /cmd_vel: %s", e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Serial port for controller not open.");
        }
    }

    LibSerial::SerialPort serial_; // Serial port object
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_; // ROS2 publisher
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_; // ROS2 subscription
    std::thread receive_thread_; // Thread for receiving data
    bool run_receive_thread_; // Flag to control the receive thread
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BidirectionalInterface>());
    rclcpp::shutdown();
    return 0;
}