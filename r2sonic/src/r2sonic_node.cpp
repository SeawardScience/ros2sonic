#include "r2sonic_node.hpp"

NS_HEAD

template<typename T>
void setupParam(T * variable,rclcpp::Node *node , std::string topic, T initial_val){
  node->declare_parameter(topic, initial_val);
  *variable =
      node->get_parameter(topic).get_parameter_value().get<T>();
}

// a explicit overload for string is required for casting to work correctly
void setupParam(std::string * variable,rclcpp::Node *node , std::string topic, std::string initial_val){
  setupParam<std::string>(variable,node,topic,initial_val);
}

void R2SonicNode::Parameters::init(rclcpp::Node *node) {
  // Helper: normalize empty IPs to "0.0.0.0"
  auto fix_empty_ip = [](std::string &ip) {
    if (ip.empty()) ip = "0.0.0.0";
  };

  // Topics
  setupParam(&topics.detections,     node, "topics.detections", topics.detections);
  setupParam(&topics.bth0,           node, "topics.bth0", topics.bth0);
  setupParam(&topics.aid0,           node, "topics.aid0", topics.aid0);
  setupParam(&topics.acoustic_image, node, "topics.acoustic_image", topics.acoustic_image);

  // Driver
  setupParam(&driver.control_mode, node, "driver.control_mode", driver.control_mode);

  // Sonar
  setupParam(&sonar.system_index,  node, "sonar.system_index", sonar.system_index);
  setupParam(&sonar.head_serial,   node, "sonar.head_serial", sonar.head_serial);
  setupParam(&sonar.sim_serial,    node, "sonar.sim_serial", sonar.sim_serial);
  setupParam(&sonar.interface_ip,  node, "sonar.interface_ip", sonar.interface_ip);
  fix_empty_ip(sonar.interface_ip);
  setupParam(&sonar.tx_frame_id,   node, "sonar.tx_frame_id", sonar.tx_frame_id);
  setupParam(&sonar.rx_frame_id,   node, "sonar.rx_frame_id", sonar.rx_frame_id);



  // Network
  setupParam(&network.subnet_mask,  node, "network.subnet_mask", network.subnet_mask);
  fix_empty_ip(network.subnet_mask);
  setupParam(&network.gateway_ip,   node, "network.gateway_ip", network.gateway_ip);
  fix_empty_ip(network.gateway_ip);
  setupParam(&network.gui_ip,       node, "network.gui_ip", network.gui_ip);
  fix_empty_ip(network.gui_ip);
  setupParam(&network.gui_baseport, node, "network.gui_baseport", network.gui_baseport);

  for (size_t i = 0; i < SYSTEMS; ++i) {
    setupParam(&network.head_ips[i], node, "network.head_ips_" + std::to_string(i), network.head_ips[i]);
    fix_empty_ip(network.head_ips[i]);
    setupParam(&network.head_baseports[i], node, "network.head_baseports_" + std::to_string(i), network.head_baseports[i]);

    setupParam(&network.simbox_ips[i], node, "network.simbox_ips_" + std::to_string(i), network.simbox_ips[i]);
    fix_empty_ip(network.simbox_ips[i]);
    setupParam(&network.simbox_baseports[i], node, "network.simbox_baseports_" + std::to_string(i), network.simbox_baseports[i]);
  }

  for (size_t i = 0; i < DAQS; ++i) {
    setupParam(&network.daq_ips[i], node, "network.daq_ips_" + std::to_string(i), network.daq_ips[i]);
    fix_empty_ip(network.daq_ips[i]);
    setupParam(&network.daq_baseports[i], node, "network.daq_baseports_" + std::to_string(i), network.daq_baseports[i]);
  }

  // Sim Commands - Int
  for (auto &[key, value_pair] : sim_commands.int_params) {
    setupParam(&value_pair.second, node, "sim_commands." + key, value_pair.second);
  }

  // Sim Commands - IP Strings
  for (auto &[key, value_pair] : sim_commands.ip_str_params) {
    setupParam(&value_pair.second, node, "sim_commands." + key, value_pair.second);
  }

  // Head Commands - Float
  for (auto &[key, value_pair] : head_commands.float_params) {
    setupParam(&value_pair.second, node, "head_commands." + key, value_pair.second);
  }

  // Head Commands - Int
  for (auto &[key, value_pair] : head_commands.int_params) {
    setupParam(&value_pair.second, node, "head_commands." + key, value_pair.second);
  }


  // Summary Log
  std::ostringstream oss;
  oss << "\n========== R2Sonic Network Configuration ==========\n"
      << " System Index        : " << sonar.system_index << "\n"
      << " Subnet Mask         : " << network.subnet_mask << "\n"
      << " Gateway IP          : " << network.gateway_ip << "\n"
      << " GUI IP              : " << network.gui_ip << "\n"
      << " GUI Baseport        : " << network.gui_baseport << "\n"
      << "---------------------------------------------------\n";

  for (size_t i = 0; i < SYSTEMS; ++i) {
    oss << " Head " << i << " IP           : " << network.head_ips[i] << "\n"
        << " Head " << i << " Baseport     : " << network.head_baseports[i] << "\n"
        << " Simbox " << i << " IP         : " << network.simbox_ips[i] << "\n"
        << " Simbox " << i << " Baseport   : " << network.simbox_baseports[i] << "\n"
        << "---------------------------------------------------\n";
  }

  for (size_t i = 0; i < DAQS; ++i) {
    oss << " DAQ " << i << " IP            : " << network.daq_ips[i] << "\n"
        << " DAQ " << i << " Baseport      : " << network.daq_baseports[i] << "\n";
  }

  oss << "===================================================\n"
      << " Selected Head IP    : " << getHeadIp() << "\n"
      << " Selected Head Port  : " << getHeadBaseport() << "\n"
      << " Selected Simbox IP  : " << getSimIp() << "\n"
      << " Selected Simbox Port: " << getSimBaseport() << "\n"
      << "===================================================";

  RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());

  // Control mode validation
  if (driver.control_mode && (sonar.head_serial.empty() || sonar.sim_serial.empty())) {
    RCLCPP_ERROR(node->get_logger(),
                 "Control mode is enabled, but sonar.head_serial or sonar.sim_serial is empty.\n"
                 "Disabling control mode.");
    driver.control_mode = false;
  }
}





// bool send_udp_message(packets::CmdSet message, const std::string& destination_ip,
//             const unsigned short port) {

//   using namespace boost::asio;
//   io_service io_service;
//   ip::udp::socket socket(io_service);
//   auto remote = ip::udp::endpoint(ip::address::from_string(destination_ip), port);
//   try {
//     socket.open(boost::asio::ip::udp::v4());
//     auto buff = message.serialize();
//     socket.send_to(buffer(buff,
//                           buff.size()),
//                           remote);

//   } catch (const boost::system::system_error& ex) {
//     return false;
//   }
//   return true;
// }

R2SonicNode::R2SonicNode():
  Node("r2sonic")
{
  parameters_.init(this);

  if(shouldAdvertise(getParams().topics.detections)){
    msg_buffer_.dectections.pub =
        this->create_publisher<marine_acoustic_msgs::msg::SonarDetections>(parameters_.topics.detections,100);
  }

  if(shouldAdvertise(getParams().topics.bth0)){
    msg_buffer_.bth0.pub =
        this->create_publisher<r2sonic_interfaces::msg::RawPacket>(parameters_.topics.bth0,100);
  }

  if(shouldAdvertise(getParams().topics.aid0)){
    msg_buffer_.aid0.pub =
        this->create_publisher<r2sonic_interfaces::msg::RawPacket>(parameters_.topics.aid0,100);
  }

  if(shouldAdvertise(getParams().topics.acoustic_image)){
    msg_buffer_.acoustic_image.pub =
        this->create_publisher<marine_acoustic_msgs::msg::RawSonarImage>(parameters_.topics.acoustic_image,100);
  }

  // Register the timer to call send_sim_cmds every second
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&R2SonicNode::send_cmds, this)
      );


  // Register parameter update callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&R2SonicNode::onParameterUpdate, this, std::placeholders::_1)
      );

}

//void R2SonicNode::publish(packets::Packet &r2_packet){
//  publish(reinterpret_cast<packets::BTH0>(&r2_packet));
//  publish(reinterpret_cast<packets::AID0>(&r2_packet));
//}

void R2SonicNode::publish(packets::BTH0 &r2_packet){
  if(!r2_packet.isType()){
    return;
  }

  msg_buffer_.dectections.lock();
  msg_buffer_.bth0.lock();

  if(shouldPublish(msg_buffer_.dectections.pub)){
    conversions::bth02SonarDetections(&msg_buffer_.dectections.msg,r2_packet);
    msg_buffer_.dectections.msg.header.frame_id = getParams().sonar.tx_frame_id;
    //msg_buffer_.dectections.msg.ping_info.rx_frame_id = getParams().rx_frame_id;
    msg_buffer_.dectections.pub->publish(msg_buffer_.dectections.msg);
  }

  if(shouldPublish(msg_buffer_.bth0.pub)){
    conversions::packet2RawPacket(&msg_buffer_.bth0.msg,&r2_packet);
    msg_buffer_.bth0.msg.frame_id = getParams().sonar.tx_frame_id;
    msg_buffer_.bth0.pub->publish(msg_buffer_.bth0.msg);
  }

  msg_buffer_.dectections.unlock();
  msg_buffer_.bth0.unlock();
}

void R2SonicNode::publish(packets::AID0 &aid0_packet){
  if(!aid0_packet.isType()){
    return;
  }
  msg_buffer_.aid0.lock();
  msg_buffer_.acoustic_image.lock();

  auto ping_no = aid0_packet.getPingNo();

  if(shouldPublish(msg_buffer_.aid0.pub)){
    conversions::packet2RawPacket(&msg_buffer_.aid0.msg,&aid0_packet);
    msg_buffer_.aid0.msg.frame_id = getParams().sonar.tx_frame_id;
    msg_buffer_.aid0.pub->publish(msg_buffer_.aid0.msg);
  }

  if(shouldPublish(msg_buffer_.acoustic_image.pub)){

    auto assembler = &msg_buffer_.acoustic_image_assemblers[ping_no];
    if(assembler->addPacket(aid0_packet)){
      assembler->sonar_image.header.frame_id = getParams().sonar.tx_frame_id;
      //msg->ping_info.rx_frame_id = getParams().rx_frame_id;
      msg_buffer_.acoustic_image.pub->publish(assembler->sonar_image);
      msg_buffer_.acoustic_image_assemblers.erase(ping_no);
      cleanMsgMap(&msg_buffer_.acoustic_image_assemblers, ping_no);
    }
  }

  msg_buffer_.aid0.unlock();
  msg_buffer_.acoustic_image.unlock();
}

rcl_interfaces::msg::SetParametersResult R2SonicNode::onParameterUpdate(const std::vector<rclcpp::Parameter> &params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";

  for (const auto &param : params) {
    try {
      const std::string &param_name = param.get_name();

      if (param_name.find("sim_commands.") == 0) {
        std::string key = param_name.substr(std::string("sim_commands.").length());

        // Integer parameter
        auto int_it = parameters_.sim_commands.int_params.find(key);
        if (int_it != parameters_.sim_commands.int_params.end()) {
          int_it->second.second = param.as_int();
          continue;
        }

        // IP string parameter
        auto ip_it = parameters_.sim_commands.ip_str_params.find(key);
        if (ip_it != parameters_.sim_commands.ip_str_params.end()) {
          const std::string &ip_candidate = param.as_string();

          try {
            (void)packets::ip_str_to_u32(ip_candidate.c_str());  // Validate IP
            ip_it->second.second = ip_candidate;
          } catch (const std::exception &e) {
            result.successful = false;
            result.reason = "Invalid IP for sim_commands." + key + ": " + e.what();

            // ROS2 warning log
            RCLCPP_WARN(this->get_logger(),
                        "Rejected IP update for sim_commands.%s: \"%s\" is not a valid IP address. %s",
                        key.c_str(), ip_candidate.c_str(), e.what());
          }

          continue;
        }

        result.successful = false;
        result.reason = "Unknown parameter in sim_commands: " + key;
      }

      else if (param_name.find("head_commands.") == 0) {
        std::string key = param_name.substr(std::string("head_commands.").length());

        auto float_it = parameters_.head_commands.float_params.find(key);
        if (float_it != parameters_.head_commands.float_params.end()) {
          float_it->second.second = param.as_double();
        } else {
          auto int_it = parameters_.head_commands.int_params.find(key);
          if (int_it != parameters_.head_commands.int_params.end()) {
            int_it->second.second = param.as_int();
          } else {
            result.successful = false;
            result.reason = "Unknown parameter in head_commands: " + key;
          }
        }
      }

      else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param_name;
      }

    } catch (const std::exception &e) {
      result.successful = false;
      result.reason = std::string("Failed to update parameter: ") + e.what();
    }
  }

  return result;
}



void R2SonicNode::send_cmds()
{
  if(parameters_.driver.control_mode){
    sendNetworkConfig();
    send_sim_cmds();
    send_head_cmds();
  }
}

void R2SonicNode::sendNetworkConfig()
{
  const auto& params = getParams();
  const auto& net = params.network;

  // Common config population lambda
  auto make_config = [&](const std::string& serial) {
    packets::R2DC cfg;
    std::strncpy(cfg.serial_number, serial.c_str(), sizeof(cfg.serial_number));

    cfg.subnet_mask   = packets::ip_str_to_u32(net.subnet_mask.c_str());
    cfg.gateway_ip    = packets::ip_str_to_u32(net.gateway_ip.c_str());
    cfg.gui_ip        = packets::ip_str_to_u32(net.gui_ip.c_str());
    cfg.gui_baseport  = net.gui_baseport;
    cfg.system        = params.sonar.system_index;

    for (size_t i = 0; i < SYSTEMS; ++i) {
      cfg.head_ip[i]         = packets::ip_str_to_u32(net.head_ips[i].c_str());
      cfg.head_baseport[i]   = net.head_baseports[i];
      cfg.simbox_ip[i]       = packets::ip_str_to_u32(net.simbox_ips[i].c_str());
      cfg.simbox_baseport[i] = net.simbox_baseports[i];
    }

    for (size_t i = 0; i < DAQS; ++i) {
      cfg.daq_ip[i]       = packets::ip_str_to_u32(net.daq_ips[i].c_str());
      cfg.daq_baseport[i] = net.daq_baseports[i];
    }

    cfg.setConfigId();
    return cfg;
  };

  // Send to head
  auto head_cfg = make_config(params.sonar.head_serial);
  sendUdpMessage(head_cfg, "10.255.255.255", 53810);

  // Send to simbox
  auto sim_cfg = make_config(params.sonar.sim_serial);
  sendUdpMessage(sim_cfg, "10.255.255.255", 53810);
}



void R2SonicNode::send_sim_cmds() {
  packets::CmdSet cmd_pkt;

  // Add all SimCmds - int values
  for (const auto &[key, value_pair] : parameters_.sim_commands.int_params) {
    const auto &cmd_name = value_pair.first;
    const auto &value = value_pair.second;
    cmd_pkt.append(packets::IntCmd(cmd_name.c_str(), value));
  }

  // Add all SimCmds - IP string values
  for (const auto &[key, value_pair] : parameters_.sim_commands.ip_str_params) {
    const auto &cmd_name = value_pair.first;
    const auto &ip_str = value_pair.second;
    uint32_t ip_u32 = packets::ip_str_to_u32(ip_str.c_str());
    cmd_pkt.append(packets::IntCmd(cmd_name.c_str(), ip_u32));
  }

  // Send the serialized packet (e.g., via UDP)
  sendUdpMessage(cmd_pkt, parameters_.getSimIp(), parameters_.getSimBaseport() + 2);

  RCLCPP_DEBUG(this->get_logger(), "Sent SimCmds (with IPs) to %s:%d",
               parameters_.getSimIp().c_str(), parameters_.getSimBaseport() + 2);
}


void R2SonicNode::send_head_cmds() {
  packets::CmdSet cmd_pkt;

  // Serialize float commands
  for (const auto &[key, value_pair] : parameters_.head_commands.float_params) {
    const auto &cmd_name = value_pair.first; // Command name (e.g., "ABS0")
    const auto &value = value_pair.second;   // Command value
    cmd_pkt.append(packets::FloatCmd(cmd_name.c_str(), value));
  }

  // Serialize integer commands
  for (const auto &[key, value_pair] : parameters_.head_commands.int_params) {
    const auto &cmd_name = value_pair.first; // Command name (e.g., "BIE0")
    const auto &value = value_pair.second;   // Command value
    cmd_pkt.append(packets::IntCmd(cmd_name.c_str(), value));
  }

  // Serialize the packet
  auto buff = cmd_pkt.serialize();

  // Send the serialized packet (e.g., via UDP)
  sendUdpMessage(cmd_pkt, parameters_.getHeadIp(), parameters_.getHeadBaseport()+2);

  // Optional: Log the operation
  RCLCPP_DEBUG(this->get_logger(), "Sent HeadCmds to sonar at %s", parameters_.getHeadIp().c_str());
}


template <typename T>
void R2SonicNode::cleanMsgMap(msgMap<T> *msg_map, u32 ping_no){
  auto iter = msg_map->begin();
  auto end_itr = msg_map->end();
  for(; iter != end_itr; ) {
    auto item_ping_no = iter->first;
    if (ping_no - 10 > item_ping_no) {
        iter = msg_map->erase(iter);
    } else {
        ++iter;
    }
  }
}

bool R2SonicNode::shouldAdvertise(std::string topic){
  return topic != "";
}

bool R2SonicNode::shouldPublish(rclcpp::PublisherBase::SharedPtr pub){
  if(!pub){
    return false;
  }
  return true;
  return pub->get_subscription_count() > 0;
}


NS_FOOT


