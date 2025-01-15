#pragma once
#include "package_defs.hpp"
#include <rclcpp/rclcpp.hpp>
#include <r2sonic/packets/all.hpp>
#include <package_defs.hpp>
#include <conversions.hpp>
#include <mutex>
#include <r2sonic_interfaces/msg/raw_packet.hpp>
#include <boost/asio.hpp>
#include <r2sonic/packets/cmd_packet.hpp>

NS_HEAD

/*!
 * \brief The rclcpp::Node representing the connection between ROS and the R2Sonic Unit
 */
class R2SonicNode : public rclcpp::Node
{
public:
  R2SonicNode();
//  void publish(packets::Packet & r2_packet);
  /*!
   * \brief Publishes all ros2 messages corresponding to a received
   * BTH0 Packet.
   * \param Publishes all ros2 messages corresponding to a received
   * BTH0 Packet.
   */
  void publish(packets::BTH0 & r2_packet);

  /*!
   * \brief Publishes all ros2 messages corresponding to a received
   * AID0 Packet.
   * \param Publishes all ros2 messages corresponding to a received
   * AID0 Packet.
   */
  void publish(packets::AID0 & aid0_packet);

  /*!
   * \brief a Structure that corresponds to the parameters advertised
   * by the R2SonicNode class.   All params assoicated with this struct
   * will have the same structure struct.  Example: topics.detections member
   * mapps to param "topics/detections"
   */
  struct Parameters{
    struct Topics{
      std::string detections;       //!< detections topic with type marine_acoustic_msgs::RawSonarDetections
      std::string bth0;             //!< raw bth0 topic r2sonic_interfaces::RawPacket
      std::string acoustic_image;   //!< acoustic image topic with type marine_acoustic_msgs::RawSonarImage
      std::string aid0;             //!< raw aid0 topic with type r2sonic_interfaces::RawPacket
    } topics; //!< a container for the topics.
    struct Ports{
      int bathy;                    //!< the port to listen to bathy
      int acoustic_image;           //!< the port to listen for acoustic image data
    } ports;  //!< a container for the ports
    std::string sonar_ip;           //!< the ip address to send commands to the sonar
    std::string sim_ip;
    std::string interface_ip;       //!< the interface you want to listen on (0.0.0.0 to listen on all)
    std::string tx_frame_id;        //!< the frame ID of the acoustic center of the transmitter
    std::string rx_frame_id;        //!< the frame ID of the acoustic center of the receiver

    struct SimCmds {
      std::map<std::string, std::pair<std::string, int>> int_params; //!< Map of integer parameters with command names and values

      SimCmds() {
        int_params = {
          int_params = {
              // Initialization and status
              {"status_data", {"STM0", 1}},         // Status data mode (on)

              {"iins", {"IINS", 1}},

              // GPS commands
              {"gps_enable", {"ENG0", 1}},          // GPS enable
              {"gps_interface", {"DRG0", 0}},       // GPS interface (RS-232)
              {"gps_baud", {"BDG0", 9600}},         // GPS baud rate
              {"gps_data_bits", {"DBG0", 8}},       // GPS data bits
              {"gps_parity", {"PAG0", 0}},          // GPS parity (none)
              {"gps_stop_bits", {"SBG0", 1}},       // GPS stop bits
              {"pps_edge", {"POG0", 0}},            // PPS edge (rising)

              // Trigger commands
              {"trigger_in_mode", {"SYI0", 0}},     // Trigger in mode (rising edge)
              {"trigger_out_mode", {"SYO0", 0}},    // Trigger out mode (off)

              // Heading commands
              {"heading_enable", {"ENH0", 0}},      // Heading enable
              {"heading_interface", {"DRH0", 2}},   // Heading interface (RS-232)

              // Motion commands
              {"motion_enable", {"ENM0", 1}},       // Motion enable
              {"motion_interface", {"DRM0", 0}},    // Motion interface (RS-232)
              {"motion_baud", {"BDM0", 115200}},    // Motion baud rate
              {"motion_data_bits", {"DBM0", 8}},    // Motion data bits
              {"motion_parity", {"PAM0", 0}},       // Motion parity (none)
              {"motion_stop_bits", {"SBM0", 1}},    // Motion stop bits

              // SVP commands
              {"svp_enable", {"ENS0", 1}},          // SVP enable
              {"svp_interface", {"DRS0", 0}},       // SVP interface (RS-232)
              {"svp_baud", {"BDS0", 19200}},        // SVP baud rate
              {"svp_data_bits", {"DBS0", 8}},       // SVP data bits
              {"svp_parity", {"PAS0", 0}},          // SVP parity (none)
              {"svp_stop_bits", {"SBS0", 1}},       // SVP stop bits

              // Sonar commands
              {"sonar_power", {"SPO0", 1}}         // Sonar head power (on)
            }
        };
      }
    }sim_commands;

    struct HeadCmds {
      std::map<std::string, std::pair<std::string, float>> float_params; //!< Map of float parameters with command names and values
      std::map<std::string, std::pair<std::string, int>> int_params; //!< Map of integer parameters with command names and values

      HeadCmds() {
        float_params = {
                        {"absorption", {"ABS0", 80.0f}},
                        {"acoustic_brightness", {"AIB0", 30.0f}},
                        {"max_range", {"BMAX", 999.0f}},
                        {"min_range", {"BMIN", 0.0f}},
                        {"sound_velocity", {"SVL0", 1496.0f}},
                        {"range", {"RNG0", 25.0f}},
                        {"receiver_tilt", {"RET0", 0.0f}},
                        {"depth_gate_min", {"DGA0", 6.8f}},
                        {"depth_gate_max", {"DGB0", 10.4f}},
                        {"depth_gate_slope", {"DGS0", 0.0f}},
                        {"frequency", {"FRQ0", 400000.0f}},
                        {"receiver_gain", {"GAN0", 14.0f}},
                        {"projector_focus", {"TXF0", 0.0f}},
                        {"transmitter_power", {"TXP0", 191.0f}},
                        {"transmitter_pulse_length", {"TXL0", 275e-6f}},
                        {"vertical_steering_angle", {"TXS0", 0.0f}},
                        {"sector_width", {"SEW0", 2.79f}},
                        {"sector_rotate", {"SER0", 0.0f}},
                        };

        int_params = {
                      {"bathy_intensity_enable", {"BIE0", 1}},
                      {"sound_velocity_enable", {"SVU0", 0}},
                      {"roll_stabilization_enable", {"ROS0", 0}},
                      {"depth_gate_mode", {"DGO0", 0}},
                      {"head_sync_mode", {"DHM0", 0}},
                      {"projector_orientation", {"PRO0", 0}},
                      {"water_column_enable", {"WCM0", 0}},
                      {"pitch_stabilization_enable", {"PTS0", 0}},
                      {"auto_mode_flags", {"AUT0", 0}},
                      {"ping_mode", {"TRG0", 0}},
                      };
      }
    }head_commands;

    /*!
     * \brief declares all the parameters and initialized all the stored variables
     * within the struct
     * \param node  A pointer or reference to the node you want to use to
     * initialize the parameters.
     */
    void init(rclcpp::Node * node);
  };

  /*!
   * \brief gets a referenence to the r2sonic::R2SonicNode::Parameters
   * \return the parameters associated with the node
   */
  const Parameters & getParams(){return parameters_;}

protected:

  /*!
     * \brief Handles parameter updates dynamically.
     * \param params List of parameters being updated.
     * \return Result indicating success or failure of parameter update.
     */
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(const std::vector<rclcpp::Parameter> &params);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_; //!< Handle for parameter callback
  Parameters parameters_;  //!< potected storage for parametrs


  rclcpp::TimerBase::SharedPtr timer_; //!< Timer to periodically send SimCmds

  void send_cmds();
  void send_sim_cmds();
  void send_head_cmds();

  /*!
   * \brief a colleciton of rosmessageg publisher and a mutex grouped for convienence.
   */
  template <typename MSG_T, typename STORAGE_T>
  struct msgMtx_{
    STORAGE_T msg;    //!< the way you want to store the message
    std::shared_ptr< rclcpp::Publisher<MSG_T> > pub; //!< the publisher to publish the message
    std::mutex mtx;   //!< the mutex associated with the message
    void lock(){mtx.lock();}
    void unlock(){mtx.unlock();}
  };

  template <typename T>
  using msgMtx = msgMtx_<T, T>; //!< Typedef for a single message, useful if you don't need to assemble multiple packets

  template <typename T>
  using msgMap = std::map<u32, T>;  //!< This typedef is useful for messages that need to be assmebled from multiple packets

  template <typename T>
  using msgMtxMap = msgMtx_< T, msgMap<T> >; //!< Creates a msgMtx with a map of messages for assembling multiple packets into one ros message

  /*!
   * \brief a container for the messages we want to buffer
   */
  struct MsgBuffer{
    msgMtx<marine_acoustic_msgs::msg::SonarDetections> dectections;
    msgMtx<r2sonic_interfaces::msg::RawPacket> bth0;

    msgMtx<marine_acoustic_msgs::msg::RawSonarImage> acoustic_image;
    std::map<u32,conversions::Aid02RawAiAssembler> acoustic_image_assemblers;
    msgMtx<r2sonic_interfaces::msg::RawPacket> aid0;
  } msg_buffer_;

  /*!
   * \brief removes incomplete messages from the map if they are too old based on
   * the ping_number
   * \param msg_map
   * \param ping_no
   */
  template <typename T>
  void cleanMsgMap(msgMap<T> *msg_map, u32 ping_no);
  /*!
   * \brief Holds the conditions that determine if the a tiopic should be advertised
   * \param topic the topic you want to test
   * \return true if you should advertise it
   */
  bool shouldAdvertise(std::string topic);
  /*!
   * \brief Checks to see if there are any subscribers to a topic before computing and publishing it
   * \param pub the publisher you want to check
   * \return true if you should publish on that publisher (e.g. has subscribers)
   */
  bool shouldPublish(rclcpp::PublisherBase::SharedPtr pub);

};


NS_FOOT
