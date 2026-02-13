#pragma once

#include "can_id.hpp"

namespace can {
/**
 * @brief CAN 1 & 2 Modules (Based on can.md)
 */
enum class CanDest : can::CanId {
  // CAN 1: Mechanism System
  central_mech = 0x000,
  power_12v    = 0x100,
  dc_lift      = 0x300,
  servo_yagura = 0x400,
  servo_ring   = 0x401,

  // CAN 2: Drive System
  central_drive = 0x001,
  power_24v     = 0x101,
  m3508_ctrl    = 0x200,
  m3508_1_fb    = 0x201,
  m3508_2_fb    = 0x202,
  m3508_3_fb    = 0x203,
  m3508_4_fb    = 0x204,
};

constexpr bool operator==(const can::CanId id, const can::CanDest dest) {
  return id == static_cast<can::CanId>(dest);
}

constexpr bool operator==(const can::CanDest dest, const can::CanId id) {
  return id == static_cast<can::CanId>(dest);
}
}  // namespace can
