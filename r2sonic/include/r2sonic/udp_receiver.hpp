#pragma once

#include "package_defs.hpp"

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>

#include <iostream>

NS_HEAD

class UdpReceiver
{
public:
  UdpReceiver();
  void receiveHandler(const boost::system::error_code& error, size_t bytes_transferred);
  virtual void receiveImpl(const boost::system::error_code& error, size_t bytes_transferred){return;}
  void wait();
  void receive(const std::string& host,
               const std::string& port);
  char * startBit();

protected:
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::array<char, 1024> recv_buffer_;
  //char recv_buffer_[1024];
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

NS_FOOT
