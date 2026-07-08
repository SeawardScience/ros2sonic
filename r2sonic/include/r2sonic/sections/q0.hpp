#pragma once
#include "sections_defs.hpp"
#include <sections/section.hpp>

SECTIONS_NS_HEAD
    /*!
 * \brief section Q0: 4-bit quality flags
 *
 * Per manual:
 *   u32 Q0_Quality[(H0_Points+7)/8];
 *   8 groups of 4 flag bits (phase detect, magnitude detect, reserved, reserved),
 *   packed left-to-right within each u32.
 */
    class Q0 : public Section{
public:
  using Section::Section;

  char * nominalType() const { return "Q0"; }

  // ---- raw access ----

  const u8* qualityWordsBE() const {
    return reinterpret_cast<const u8*>(start_bit_ + sizeof(SectionInfo));
  }

  // Read a big-endian u32 from the packed stream (no alignment assumptions).
  static inline u32 read_be_u32(const u8* p) {
    return (static_cast<u32>(p[0]) << 24) |
           (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) <<  8) |
           (static_cast<u32>(p[3]) <<  0);
  }

  /*!
   * \brief Return the 4-bit quality nibble for a given beam/point index.
   *
   * Word contains 8 nibbles, packed left-to-right:
   *   beam 0 -> bits 31..28
   *   beam 1 -> bits 27..24
   *   ...
   *   beam 7 -> bits  3..0
   */
  u8 getQualityFlag(u16 beam_no) const {
    const u32 word_index = static_cast<u32>(beam_no / 8u);
    const u32 nib_index  = static_cast<u32>(beam_no % 8u);

    const u8* word_ptr = qualityWordsBE() + (word_index * 4u);
    const u32 w = read_be_u32(word_ptr);

    const u32 shift = (7u - nib_index) * 4u;     // left-to-right packing
    return static_cast<u8>((w >> shift) & 0x0Fu);
  }

  // ---- per-flag helpers ----
  // Manual lists nibble bits in order: phase, magnitude, reserved, reserved (left-to-right).
  // That corresponds to bit3=phase, bit2=magnitude, bit1..0 reserved.
  bool phaseDetect(u16 beam_no) const {
    return (getQualityFlag(beam_no) & 0x08u) != 0u;
  }

  bool magnitudeDetect(u16 beam_no) const {
    return (getQualityFlag(beam_no) & 0x04u) != 0u;
  }

} __attribute__((packed));

SECTIONS_NS_FOOT
