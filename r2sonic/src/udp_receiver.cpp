#include "udp_receiver.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include <thread>

NS_HEAD

UdpReceiver::UdpReceiver():
  socket_(io_service_)
{

}

void UdpReceiver::wait(){
  socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
  remote_endpoint_,
  boost::bind(&UdpReceiver::receiveHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UdpReceiver::receive(const std::string& host,
                          const std::string& port){

  boost::asio::ip::udp::endpoint local_endpoint(
      boost::asio::ip::udp::v4(),
      std::atoi(port.c_str())
      );

  socket_.open(boost::asio::ip::udp::v4());

  // Retry logic for bind
  int retry_count = 0;
  int retry_delay_ms = 1000;

  while (rclcpp::ok()) {
    try {
      socket_.bind(local_endpoint);
      if (retry_count > 0) {
        RCLCPP_INFO(rclcpp::get_logger("r2sonic_node"),
                    "Successfully bound to %s:%s after %d attempts",
                    host.c_str(), port.c_str(), retry_count);
      } else {
        RCLCPP_INFO(rclcpp::get_logger("r2sonic_node"),
                    "Successfully bound to %s:%s",
                    host.c_str(), port.c_str());
      }
      break;
    }
    catch (const boost::system::system_error& e) {
      retry_count++;

      // Log warning every 10 attempts to avoid log spam
      if (retry_count % 10 == 1) {
        RCLCPP_WARN(rclcpp::get_logger("r2sonic_node"),
                    "Still trying to bind to %s:%s (attempt %d): %s",
                    host.c_str(), port.c_str(), retry_count, e.what());
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
    }
  }

  // Check if we exited due to shutdown
  if (!rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("r2sonic_node"),
                "Shutdown requested before successful bind to %s:%s",
                host.c_str(), port.c_str());
    return;  // or throw, depending on your error handling strategy
  }

  wait();
  io_service_.run();

}

void UdpReceiver::receive(const std::string &host, const int &port){
  receive(host, std::to_string(port));
}

void UdpReceiver::receiveHandler(const boost::system::error_code &error, size_t bytes_transferred){
  if (error) {
    std::cout << "Receive failed: " << error.message() << "\n";
    return;
  }
  receiveImpl(error, bytes_transferred);

  wait();
}

char * UdpReceiver::startBit(){
  return reinterpret_cast<char *>(&recv_buffer_);
}


NS_FOOT
