#include <rclcpp/rclcpp.hpp>
#include <rtcm_msgs/msg/message.hpp>
#include <io_interfaces/msg/raw_packet.hpp>

class RTCMToRawPacket : public rclcpp::Node
{
public:
  RTCMToRawPacket()
      : Node("rtcm_to_rawpacket_adapter")
  {
    using std::placeholders::_1;

    sub_ = this->create_subscription<rtcm_msgs::msg::Message>(
        "rtcm", 10, std::bind(&RTCMToRawPacket::callback, this, _1));

    pub_ = this->create_publisher<io_interfaces::msg::RawPacket>(
        "rtcm/to_device", 10);
  }

private:
  void callback(const rtcm_msgs::msg::Message::SharedPtr msg)
  {
    io_interfaces::msg::RawPacket packet;
    packet.header = msg->header;
    packet.data = msg->message;

    pub_->publish(packet);
  }

  rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr sub_;
  rclcpp::Publisher<io_interfaces::msg::RawPacket>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTCMToRawPacket>());
  rclcpp::shutdown();
  return 0;
}
