#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <string>
#include <sstream>

class UltrasonicInterface : public rclcpp::Node
{
public:
    UltrasonicInterface()
    : Node("ultrasonic_interface")
    {
        // Setup serial connection
        try {
            serial_.Open("/dev/esp32"); // Open the serial port
            serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); // Set baud rate
            serial_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8); // Set character size
            serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1); // Set stop bits
            serial_.SetParity(LibSerial::Parity::PARITY_NONE); // Set parity
            RCLCPP_INFO(this->get_logger(), "Serial port for ultrasonic sensor opened successfully.");
        } catch (const LibSerial::OpenFailed&) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port for ultrasonic sensor.");
            rclcpp::shutdown();
            return;
        }

        // Create publisher for ultrasonic sensor data
        pub_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic_distance", 10);

        // Start a timer to read data periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&UltrasonicInterface::readSensorData, this));
    }

    ~UltrasonicInterface()
    {
        if (serial_.IsOpen()) {
            serial_.Close(); // Close the serial port
        }
    }

private:
    void readSensorData()
    {
        if (serial_.IsOpen()) {
            try {
                std::string data;
                serial_.ReadLine(data, '\n'); // Read a line of data from the serial port

                // Convert the string data to a float
                std::istringstream iss(data);
                float distance;
                iss >> distance;

                // Publish the distance data
                auto msg = std_msgs::msg::Float32();
                msg.data = distance;
                pub_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Published distance: %.2f", distance);
            } catch (const LibSerial::ReadTimeout&) {
                RCLCPP_WARN(this->get_logger(), "Read timeout from ultrasonic sensor.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Serial port for ultrasonic sensor not open.");
        }
    }

    LibSerial::SerialPort serial_; // Serial port object
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_; // ROS2 publisher
    rclcpp::TimerBase::SharedPtr timer_; // Timer for periodic sensor reading
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicInterface>());
    rclcpp::shutdown();
    return 0;
}