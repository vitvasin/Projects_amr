#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
class LedInterface : public rclcpp::Node
{
public:
    LedInterface()
    : Node("led_interface")
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

        // Subscribe to boolean topic
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "led_toggle", 10,
            std::bind(&LedInterface::callback, this, std::placeholders::_1));
    }

    ~LedInterface()
    {
        if (serial_.IsOpen()) {
            serial_.Close(); // Close the serial port
        }
    }

private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (serial_.IsOpen()) {
            std::string val = msg->data ? "1\n" : "0\n"; // Convert boolean to string
            
            serial_.Write(val); // Write to the serial port
            RCLCPP_INFO(this->get_logger(), "Sent to ESP32: %s", val.c_str());
           
        } else {
            RCLCPP_WARN(this->get_logger(), "Serial port not open.");
        }
    }

    LibSerial::SerialPort serial_; // Serial port object
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_; // ROS2 subscription
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedInterface>());
    rclcpp::shutdown();
    return 0;
}