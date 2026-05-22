// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>



namespace rm_serial_driver
{
struct ReceivePacket
{//ypr
  uint8_t header = 0xEE;
 
  uint8_t checksum = 0xED;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xEE;
  //各电机的电流设置
  uint16_t wheel_lf_current;
  uint16_t wheel_rf_current;
  uint16_t wheel_lb_current;
  uint16_t wheel_rb_current;
  uint8_t checksum = 0xED;
} __attribute__((packed));



inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
