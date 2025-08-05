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

void R2SonicNode::Parameters::init(rclcpp::Node *node){
  setupParam(&topics.detections,node,"topics.detections","~/detections");
  setupParam(&topics.bth0,node,"topics.bth0","~/raw/bth0");
  setupParam(&topics.aid0,node,"topics.aid0","~/raw/aid0");
  setupParam(&topics.acoustic_image,node,"topics.acoustic_image","~/acoustic_image");
  setupParam(&ports.bathy,node,"ports.bathy",4000);
  setupParam(&ports.acoustic_image,node,"ports.acoustic_image" ,4003);
  setupParam(&sonar_ip,node,"sonar_ip","10.0.0.86");
  setupParam(&sim_ip,node,"sim_ip","10.0.0.99");
  setupParam(&interface_ip,node,"interface_ip","0.0.0.0");
  setupParam(&tx_frame_id,node,"tx_frame_id","r2sonic_tx");
  setupParam(&rx_frame_id,node,"rx_frame_id","r2sonic_rx");

  // SimCmds
  for (auto &[key, value_pair] : sim_commands.int_params) {
    setupParam(&value_pair.second, node, "sim_commands." + key, value_pair.second);
  }

  // HeadCmds (Float)
  for (auto &[key, value_pair] : head_commands.float_params) {
    setupParam(&value_pair.second, node, "head_commands." + key, value_pair.second);
  }

  // HeadCmds (Int)
  for (auto &[key, value_pair] : head_commands.int_params) {
    setupParam(&value_pair.second, node, "head_commands." + key, value_pair.second);
  }


  RCLCPP_INFO(node->get_logger(), "Listening on interface  : %s", interface_ip.c_str());
  RCLCPP_INFO(node->get_logger(), "Sending sonar comands on: %s", sonar_ip.c_str());


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
    msg_buffer_.dectections.msg.header.frame_id = getParams().tx_frame_id;
    //msg_buffer_.dectections.msg.ping_info.rx_frame_id = getParams().rx_frame_id;
    msg_buffer_.dectections.pub->publish(msg_buffer_.dectections.msg);
  }

  if(shouldPublish(msg_buffer_.bth0.pub)){
    conversions::packet2RawPacket(&msg_buffer_.bth0.msg,&r2_packet);
    msg_buffer_.bth0.msg.frame_id = getParams().tx_frame_id;
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
    msg_buffer_.aid0.msg.frame_id = getParams().tx_frame_id;
    msg_buffer_.aid0.pub->publish(msg_buffer_.aid0.msg);
  }

  if(shouldPublish(msg_buffer_.acoustic_image.pub)){

    auto assembler = &msg_buffer_.acoustic_image_assemblers[ping_no];
    if(assembler->addPacket(aid0_packet)){
      assembler->sonar_image.header.frame_id = getParams().tx_frame_id;
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

      // Check if the parameter belongs to SimCmds
      if (param_name.find("sim_commands.") == 0) {
        std::string key = param_name.substr(std::string("sim_commands.").length());
        auto it = parameters_.sim_commands.int_params.find(key);
        if (it != parameters_.sim_commands.int_params.end()) {
          it->second.second = param.as_int();
        } else {
          result.successful = false;
          result.reason = "Unknown parameter in sim_commands: " + key;
        }
      }
      // Check if the parameter belongs to HeadCmds (Float)
      else if (param_name.find("head_commands.") == 0) {
        std::string key = param_name.substr(std::string("head_commands.").length());

        // Check in float_params
        auto float_it = parameters_.head_commands.float_params.find(key);
        if (float_it != parameters_.head_commands.float_params.end()) {
          float_it->second.second = param.as_double();
        }
        // Check in int_params
        else {
          auto int_it = parameters_.head_commands.int_params.find(key);
          if (int_it != parameters_.head_commands.int_params.end()) {
            int_it->second.second = param.as_int();
          } else {
            result.successful = false;
            result.reason = "Unknown parameter in head_commands: " + key;
          }
        }
      }
      // Unknown parameter
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
  sendNetworkConfig();
  send_sim_cmds();
  send_head_cmds();
}

void R2SonicNode::sendNetworkConfig()
{
  packets::R2DC head("300541");
  head.setConfigId(this->get_name());
  sendUdpMessage(head, "10.255.255.255", 53810);

  packets::R2DC sim("200674");
  sim.setConfigId(this->get_name());
  sendUdpMessage(sim,  "10.255.255.255", 53810);
}

void R2SonicNode::send_sim_cmds() {
  packets::CmdSet cmd_pkt;

  // Add all SimCmds to the packet
  for (const auto &[key, value_pair] : parameters_.sim_commands.int_params) {
    const auto &cmd_name = value_pair.first; // Command name (e.g., "GPSB")
    const auto &value = value_pair.second;   // Command value
    cmd_pkt.append(packets::IntCmd(cmd_name.c_str(), value));
  }

  // Send the serialized packet (e.g., via UDP)
  sendUdpMessage(cmd_pkt, parameters_.sim_ip, 65502);

  // Optional: Log the operation
  RCLCPP_INFO(this->get_logger(), "Sent SimCmds to sonar at %s", parameters_.sim_ip.c_str());
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
  sendUdpMessage(cmd_pkt, parameters_.sonar_ip, 65502);

  // Optional: Log the operation
  RCLCPP_INFO(this->get_logger(), "Sent HeadCmds to sonar at %s", parameters_.sonar_ip.c_str());
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


