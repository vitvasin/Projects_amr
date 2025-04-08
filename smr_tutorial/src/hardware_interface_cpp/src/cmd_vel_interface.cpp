#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>
#include <vector>
#include <iostream>
#include <iomanip>

#define HEAD 0xAA
#define DEVICE_ID 0x01
#define FUNC_MOTION 0x02

class CmdVelInterface : public rclcpp::Node
{
public:
    CmdVelInterface()
    : Node("cmd_vel_interface")
    {
        // Setup serial connection
        try {
            serial_.Open("/dev/esp32"); // Open the serial port
            serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); // Set baud rate
            serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8); // Set character size
            serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1); // Set stop bits
            serial_.SetParity(LibSerial::Parity::PARITY_NONE); // Set parity
            RCLCPP_INFO(this->get_logger(), "Serial port for controller opened successfully.");
        } catch (const LibSerial::OpenFailed&) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port for controller.");
            rclcpp::shutdown();
            return;
        }

        // Subscribe to /cmd_vel topic
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelInterface::cmdVelCallback, this, std::placeholders::_1));
    }

    ~CmdVelInterface()
    {
        if (serial_.IsOpen()) {
            serial_.Close(); // Close the serial port
        }
    }

private:
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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_; // ROS2 subscription
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelInterface>());
    rclcpp::shutdown();
    return 0;
}