// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rm_serial_driver/crc.hpp"

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0xA5;
  uint8_t flags = 0;
  uint8_t data[16] = {};
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t flags = 0;
  uint8_t data[16] = {};
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

inline float parseRoll(const ReceivePacket & _packet)
{
  float roll = 0.0F;
  std::memcpy(&roll, _packet.data, sizeof(roll));
  return roll;
}

inline std::vector<uint8_t> toVector(const SendPacket & _data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&_data),
    reinterpret_cast<const uint8_t *>(&_data) + sizeof(SendPacket), packet.begin());
  return packet;
}

inline std::vector<uint8_t> makeDyawPacket(float _dyaw)
{
  SendPacket packet{};
  std::copy(
    reinterpret_cast<const uint8_t *>(&_dyaw),
    reinterpret_cast<const uint8_t *>(&_dyaw) + sizeof(_dyaw), packet.data);
  crc16::Append_CRC16_Check_Sum(
    reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  return toVector(packet);
}

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
