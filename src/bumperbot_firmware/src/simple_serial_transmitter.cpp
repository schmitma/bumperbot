#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node
{
 public:
  SimpleSerialTransmitter() : Node("simple_serial_transmitter")
  {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    port_ = get_parameter("port").as_string();
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_1152000);
    sub_ = create_subscription<std_msgs::msg::String>(
        "serial_transmitter", 10, std::bind(&SimpleSerialTransmitter::msgCallback, this, _1));
  }

  ~SimpleSerialTransmitter()
  {
    arduino_.Close();
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  LibSerial::SerialPort arduino_;
  std::string port_;

  void msgCallback(const std_msgs::msg::String msg)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Sending data: " << msg.data);
    arduino_.Write(msg.data);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialTransmitter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}