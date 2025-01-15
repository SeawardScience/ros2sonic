#pragma once

#include "packets_defs.hpp"
#include <r2sonic/sections/primatives.hpp>
#include <cstring>    // For std::memcpy
#include <vector>     // For std::vector
#include <stdexcept>  // For exceptions
#include <variant>    // For std::variant

PACKETS_NS_HEAD

    /*!
 * \brief A structure representing an R2Sonic command.
 * \tparam T The type of the command value (e.g., BE_u32, BE_f32).
 */
    template <typename T>
    struct Cmd {
  char name[4]; //!< The 4-character command name.
  T value;      //!< The command value.

  /*!
     * \brief Constructs a command with the specified name and value.
     * \param cmd_name The 4-character command name.
     * \param cmd_value The command value.
     * \throws std::invalid_argument If the command name is not exactly 4 characters.
     */
  template <typename IN_T>
  Cmd(const char *cmd_name, const IN_T &cmd_value) {
    if (std::strlen(cmd_name) != 4) {
      throw std::invalid_argument("Command name must be exactly 4 characters.");
    }
    std::memcpy(name, cmd_name, 4);
    value = cmd_value;
  }
} __attribute__((packed));

// Type aliases for common command types
typedef Cmd<BE_f32> FloatCmd; //!< Command with a BE_f32 (Big-Endian float) value.
typedef Cmd<BE_u32> IntCmd;   //!< Command with a BE_u32 (Big-Endian uint32) value.

// Variant type to support multiple command types
using CmdVariant = std::variant<Cmd<BE_u32>, Cmd<BE_f32>>;

/*!
 * \brief A class representing a packet of R2Sonic commands.
 */
class CmdSet {
public:
  /*!
     * \brief Constructs a command packet with the fixed name 'CMD0'.
     */
  CmdSet() {
    std::memcpy(packet_name, "CMD0", 4);
  }

  /*!
     * \brief Appends a `Cmd` to the packet.
     * \param cmd The command to append (as a `CmdVariant`).
     */
  void append(const CmdVariant &cmd) {
    commands.push_back(cmd);
  }

  /*!
     * \brief Serializes the packet into a byte array.
     * \return A `std::vector<uint8_t>` containing the serialized packet.
     */
  std::vector<uint8_t> serialize() const {
    std::vector<uint8_t> buffer;

    // Append packet name
    buffer.insert(buffer.end(), packet_name, packet_name + sizeof(packet_name));

    // Append each command
    for (const auto &cmd_variant : commands) {
      std::visit([&buffer](auto &&cmd) {
        // Serialize command name
        buffer.insert(buffer.end(), cmd.name, cmd.name + sizeof(cmd.name));

        // Serialize command value
        buffer.insert(buffer.end(),
                      reinterpret_cast<const uint8_t *>(&cmd.value),
                      reinterpret_cast<const uint8_t *>(&cmd.value) + sizeof(cmd.value));
      }, cmd_variant);
    }

    return buffer;
  }

private:
  char packet_name[4];                 //!< Fixed packet name 'CMD0'
  std::vector<CmdVariant> commands; //!< Storage for the commands in the packet
};

PACKETS_NS_FOOT
