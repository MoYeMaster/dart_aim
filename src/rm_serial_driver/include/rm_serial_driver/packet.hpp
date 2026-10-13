// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>
#include <vector>

#include "rm_serial_driver/crc.hpp"

namespace rm_serial_driver
{
inline constexpr std::size_t kDataSize = 16;
inline constexpr std::size_t kFloatCount = 4;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
  "Serial protocol requires 32-bit IEEE 754 floats");

struct ReceivePacket
{
  uint8_t header = 0xA5;
  uint8_t flags = 0;
  uint8_t data[kDataSize] = {};
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t flags = 0;
  uint8_t data[kDataSize] = {};
  uint16_t checksum = 0;
} __attribute__((packed));

static_assert(sizeof(ReceivePacket) == 20, "ReceivePacket must be 20 bytes");
static_assert(sizeof(SendPacket) == 20, "SendPacket must be 20 bytes");

inline ReceivePacket fromVectorWithoutHeader(const std::vector<uint8_t> & _data)
{
  ReceivePacket packet{};
  packet.header = 0xA5;
  std::copy_n(_data.begin(), std::min(_data.size(), sizeof(ReceivePacket) - 1),
    reinterpret_cast<uint8_t *>(&packet) + 1);
  return packet;
}

// Slots 0..3 occupy payload bytes 0..3, 4..7, 8..11, and 12..15.
// memcpy avoids unaligned float access in the packed serial frame.
template<std::size_t Index, typename Packet>
inline float readFloat(const Packet & _packet)
{
  static_assert(Index < kFloatCount, "Float slot must be in [0, 3]");
  float value = 0.0F;
  std::memcpy(&value, _packet.data + Index * sizeof(float), sizeof(value));
  return value;
}

template<std::size_t Index, typename Packet>
inline void writeFloat(Packet & _packet, float value)
{
  static_assert(Index < kFloatCount, "Float slot must be in [0, 3]");
  std::memcpy(_packet.data + Index * sizeof(float), &value, sizeof(value));
}

inline std::vector<uint8_t> toVector(const SendPacket & _data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&_data),
    reinterpret_cast<const uint8_t *>(&_data) + sizeof(SendPacket), packet.begin());
  return packet;
}

// Values fill consecutive float slots; unused slots and flags remain zero.
// Byte order is unchanged from the existing protocol (native byte order).
template<typename... Floats>
inline std::vector<uint8_t> makeDataPacket(Floats... values)
{
  static_assert(sizeof...(Floats) > 0 && sizeof...(Floats) <= kFloatCount,
    "A data packet must contain 1 to 4 floats");
  static_assert((std::is_same_v<Floats, float> && ...),
    "Data packet values must be float (use F-suffixed literals)");
  SendPacket packet{};
  const std::array<float, sizeof...(Floats)> data{values...};
  std::memcpy(packet.data, data.data(), sizeof(float) * data.size());
  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  return toVector(packet);
}

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
