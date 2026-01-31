#pragma once

#include <Arduino.h>

namespace can {
using CAN_rx_message_t = struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
};
}  // namespace can
