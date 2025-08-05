#pragma once

#include "packets_defs.hpp"
#include <r2sonic/sections/primatives.hpp>
#include <cstring>
#include <arpa/inet.h> // for inet_pton
#include <string_view>
#include <vector>

#define SYSTEMS 4 ///< Maximum number of sonar systems working together
#define DAQS 8    ///< Maximum number of data destinations

PACKETS_NS_HEAD

    /*!
 * \brief Convert a dotted IPv4 address string to a 32-bit unsigned integer (host-endian).
 * \param ip_str The dotted-quad IPv4 string (e.g., "10.0.0.86")
 * \return The 32-bit unsigned integer representation of the IP address
 * \throws std::runtime_error if the string is not a valid IP address
 */
    inline u32 ip_str_to_u32(const char* ip_str) {
  u32 ip_be = 0;
  if (inet_pton(AF_INET, ip_str, &ip_be) != 1) {
    throw std::runtime_error("Invalid IP string");
  }
  return ntohl(ip_be); // convert from network (big-endian) to host-endian
}

/*!
 * \brief Simple FNV-1a hash implementation (32-bit) for binary data.
 * \param data Pointer to the data block
 * \param len Length of the data block in bytes
 * \return 32-bit FNV-1a hash
 */
inline uint32_t fnv1a_hash(const void* data, size_t len) {
  constexpr uint32_t FNV_OFFSET = 0x811C9DC5;
  constexpr uint32_t FNV_PRIME  = 0x01000193;

  uint32_t hash = FNV_OFFSET;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);

  for (size_t i = 0; i < len; ++i) {
    hash ^= bytes[i];
    hash *= FNV_PRIME;
  }

  return hash;
}

/*!
 * \brief Represents an R2Sonic Device Configuration (R2DC) packet.
 */
struct R2DC {
  char name[4];                //!< Always "R2DC", 4 characters, not null-terminated
  char serial_number[12];      //!< Serial number string, null-padded if shorter than 12 characters
  BE_u32 config_id;            //!< Unique config identifier used for sync with R2DS
  BE_u32 subnet_mask;          //!< Network subnet mask
  BE_u32 gateway_ip;           //!< Gateway IP address (0.0.0.0 if none)
  BE_u32 gui_ip;               //!< IP address of the control GUI
  BE_u16 gui_baseport;         //!< UDP base port used by the GUI
  BE_u16 system;               //!< Head or SIM index (0 to SYSTEMS-1)
  BE_u32 head_ip[SYSTEMS];     //!< IP addresses of sonar heads
  BE_u16 head_baseport[SYSTEMS]; //!< Base UDP ports for sonar heads
  BE_u32 simbox_ip[SYSTEMS];   //!< IP addresses of simboxes
  BE_u16 simbox_baseport[SYSTEMS]; //!< Base UDP ports for simboxes
  BE_u32 daq_ip[DAQS];         //!< IP addresses for DAQ data destinations
  BE_u16 daq_baseport[DAQS];   //!< UDP ports for DAQ data destinations
  BE_u32 spare[8];             //!< Reserved for future use

  /*!
   * \brief Constructor that initializes the R2DC packet with default network configuration and a serial number.
   * \param serial_num Serial number string (null-padded to 12 characters if shorter)
   */
  explicit R2DC(const char* serial_num) {
    std::memcpy(name, "R2DC", 4);
    std::memset(serial_number, 0, sizeof(serial_number));
    if (serial_num != nullptr) {
      std::strncpy(serial_number, serial_num, sizeof(serial_number));
    }

    config_id = 2549138156;  // Temporary default (will be overridden by setConfigID)

    subnet_mask = ip_str_to_u32("255.0.0.0");
    gateway_ip  = ip_str_to_u32("0.0.0.0");
    gui_ip      = ip_str_to_u32("10.0.1.102");
    gui_baseport = 65400;
    system = 0;

    head_ip[0] = ip_str_to_u32("10.0.0.86");
    head_ip[1] = ip_str_to_u32("10.0.1.86");
    head_ip[2] = ip_str_to_u32("0.0.0.0");
    head_ip[3] = ip_str_to_u32("0.0.0.0");

    head_baseport[0] = 65500;
    head_baseport[1] = 65500;
    head_baseport[2] = 0;
    head_baseport[3] = 0;

    simbox_ip[0] = ip_str_to_u32("10.0.0.99");
    simbox_ip[1] = ip_str_to_u32("10.0.1.99");
    simbox_ip[2] = ip_str_to_u32("0.0.0.0");
    simbox_ip[3] = ip_str_to_u32("0.0.0.0");

    simbox_baseport[0] = 65500;
    simbox_baseport[1] = 65500;
    simbox_baseport[2] = 0;
    simbox_baseport[3] = 0;

    daq_ip[0] = ip_str_to_u32("10.0.1.102");
    daq_ip[1] = ip_str_to_u32("10.0.1.102");
    daq_ip[2] = ip_str_to_u32("10.0.1.102");
    for (int i = 3; i < DAQS; ++i)
      daq_ip[i] = ip_str_to_u32("0.0.0.0");

    daq_baseport[0] = 4100;
    daq_baseport[1] = 4200;
    daq_baseport[2] = 4300;
    for (int i = 3; i < DAQS; ++i)
      daq_baseport[i] = 0;

    for (int i = 0; i < 8; ++i)
      spare[i] = 0;

  }

  /*!
   * \brief Generates a unique 32-bit configuration ID based on current settings and node name.
   * \param node_name The ROS 2 node name to tie the config ID to this software instance
   * \return 32-bit hash of the configuration + node name
   */
  u32 generateConfigID(std::string_view node_name) const {
    uint32_t hash = fnv1a_hash(node_name.data(), node_name.size());
    const uint8_t* config_start = reinterpret_cast<const uint8_t*>(&subnet_mask);
    size_t config_size = sizeof(R2DC) - offsetof(R2DC, subnet_mask);
    hash ^= fnv1a_hash(config_start, config_size);
    return hash;
  }

  /*!
   * \brief Sets the internal config_id field using the current configuration and node name.
   * \param node_name The ROS 2 node name to tie the config ID to this software instance
   */
  void setConfigId(std::string_view node_name) {
    config_id = generateConfigID(node_name);
  }

  /*!
   * \brief Validates the current config_id against the expected one for the given node name.
   * \param node_name The ROS 2 node name
   * \return true if config_id matches the hash of current config + node name
   */
  bool configIDIsValid(std::string_view node_name) const {
    return config_id.get() == generateConfigID(node_name);
  }

  /*!
 * \brief Serializes the R2DC packet into a raw byte buffer suitable for UDP transmission.
 * \return std::vector<uint8_t> containing the packed data
 */
  std::vector<uint8_t> serialize() const {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(this);
    return std::vector<uint8_t>(ptr, ptr + sizeof(R2DC));
  }

} __attribute__((packed));

PACKETS_NS_FOOT
