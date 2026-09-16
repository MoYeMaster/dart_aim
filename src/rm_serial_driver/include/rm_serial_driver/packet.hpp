// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

#include "rm_serial_driver/crc.hpp"

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  // uint8_t task_mode : 2;    // 0-armor 1-small_buff 2-large-buff
  // bool reset_tracker : 1;
  // uint8_t is_play : 1;
  // uint8_t reserved : 2;
  float roll;
  // float pitch;
  // float yaw;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  uint8_t flags = 0;
  uint8_t data[16] = {};
  uint16_t checksum = 0;
} __attribute__((packed));

static_assert(sizeof(SendPacket) == 20, "SendPacket must be 20 bytes");

inline ReceivePacket fromVectorWithoutHeader(const std::vector<uint8_t> & _data)
{
  ReceivePacket packet;
  packet.header = 0x5A;
  std::copy(_data.begin(), _data.end(), (reinterpret_cast<uint8_t *>(&packet) + 1));
  return packet;
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
