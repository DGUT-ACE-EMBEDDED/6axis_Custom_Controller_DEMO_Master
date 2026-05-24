// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rm_serial_driver
{

struct ReceivePacket
{
  uint8_t header = 0xEE;   // 0
  float x;                  // 1-4   position x (m)
  float y;                  // 5-8   position y (m)
  float z;                  // 9-12  position z (m)
  float qx;                 // 13-16 quaternion x
  float qy;                 // 17-20 quaternion y
  float qz;                 // 21-24 quaternion z
  float qw;                 // 25-28 quaternion w
  uint8_t checksum = 0xED;  // 29    XOR of bytes 0-28
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xEE;
  uint16_t wheel_lf_current;
  uint16_t wheel_rf_current;
  uint16_t wheel_lb_current;
  uint16_t wheel_rb_current;
  uint8_t checksum = 0xED;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::memcpy(&packet, data.data(), std::min(data.size(), sizeof(ReceivePacket)));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::memcpy(packet.data(), &data, sizeof(SendPacket));
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
