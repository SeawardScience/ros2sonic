#ifndef MINIHEADER_H
#define MINIHEADER_H

#include <types/types_defs.hpp>

TYPES_NS_HEAD

struct MiniHeader
{
  char  PacketName[4];
  u32  PacketSize;
  u32  DataStreamID;
  MiniHeader swapEndian() const{
    MiniHeader out;
    std::copy(std::begin(PacketName), std::end(PacketName), std::begin(out.PacketName));
    out.PacketSize = revPrimative<u32>(PacketSize);
    out.DataStreamID = revPrimative<u32>(DataStreamID);
    return out;
  }
}__attribute__((packed));

TYPES_NS_FOOT

#endif // MINIHEADER_H
