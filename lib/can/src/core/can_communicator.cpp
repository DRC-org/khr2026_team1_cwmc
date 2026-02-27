#include "can_communicator.hpp"

#include <Arduino.h>

namespace can {
constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

CanCommunicator::CanCommunicator() : receive_event_listeners() {}

void CanCommunicator::setup(twai_filter_config_t filter_config) {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 20;
  // 4モータが最大1kHzで応答するため3ms周期のポーリングで最大12フレーム到達しうる
  g_config.rx_queue_len = 30;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

  if (twai_driver_install(&g_config, &t_config, &filter_config) == ESP_OK) {
    Serial.println("Driver install OK!");
  } else {
    Serial.println("Driver install fail!");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("Node enable OK!");
  } else {
    Serial.println("Node enable fail!");
    return;
  }
}

void CanCommunicator::transmit(const CanTxMessage message) const {
  twai_message_t tx_msg = {};
  tx_msg.identifier = message.id;
  tx_msg.data_length_code = 8;
  tx_msg.extd = 0;  // 標準フレーム (11-bit ID)
  tx_msg.rtr = 0;
  for (int i = 0; i < 8; i++) {
    tx_msg.data[i] = message.data[i];
  }
  // ControlTask の 3ms 周期を守るためノンブロッキング送信
  twai_transmit(&tx_msg, 0);
}

void CanCommunicator::receive() {
  twai_message_t rx_msg;
  // RX キューに溜まったフレームをすべて処理
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (rx_msg.rtr) continue;

    std::array<uint8_t, 8> data_array = {};
    uint8_t len = rx_msg.data_length_code < 8 ? rx_msg.data_length_code : 8;
    for (uint8_t i = 0; i < len; i++) {
      data_array[i] = rx_msg.data[i];
    }

    for (const auto& listener_pair : receive_event_listeners) {
      const auto& target_ids = listener_pair.first;
      const auto& callback = listener_pair.second;

      bool match = target_ids.empty();
      if (!match) {
        for (const auto& id : target_ids) {
          if (id == rx_msg.identifier) {
            match = true;
            break;
          }
        }
      }

      if (match && callback) {
        callback(rx_msg.identifier, data_array);
      }
    }
  }
}

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
        listener) {
  receive_event_listeners.push_back({listening_can_ids, listener});
}

}  // namespace can
