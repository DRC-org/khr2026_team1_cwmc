#include "can_communicator.hpp"

namespace can {
/// @brief デバッグ出力の間隔(ループ回数)
constexpr uint8_t DEBUG_PRINT_INTERVAL = 10;

constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

CanCommunicator::CanCommunicator() : receive_event_listeners() {}

void CanCommunicator::setup(twai_filter_config_t filter_config) {
  twai_onchip_node_config_t node_config = {};
  node_config.io_cfg.tx = CAN_TX;
  node_config.io_cfg.rx = CAN_RX;
  node_config.bit_timing.bitrate = 1000000;  // 1 Mbps
  node_config.tx_queue_depth = 0;            // 送信キューを無効化

  // TWAI コントローラのインスタンスを作成
  if (twai_new_node_onchip(&node_config, &node_hdl) == ESP_OK) {
    Serial.println("Node create OK!");
  } else {
    Serial.println("Node create fail!");
    return;
  }

  // receive イベントを登録
  twai_event_callbacks_t user_cbs = {};
  user_cbs.on_rx_done = &CanCommunicator::receive;

  if (twai_node_register_event_callbacks(node_hdl, &user_cbs, this) == ESP_OK) {
    Serial.println("Register receive event OK!");
  } else {
    Serial.println("Register receive event fail!");
    return;
  }

  // TWAI コントローラを有効化
  if (twai_node_enable(node_hdl) == ESP_OK) {
    Serial.println("Node enable OK!");
  } else {
    Serial.println("Node enable fail!");
    return;
  }
}

void CanCommunicator::transmit(const CanTxMessage message) const {
  static uint32_t count = 0;

  twai_frame_t tx_msg = {};
  tx_msg.header.id = message.id;
  tx_msg.header.ide = false;
  tx_msg.header.rtr = false;
  tx_msg.header.dlc = 8;
  tx_msg.buffer_len = sizeof(message.data);

  for (size_t i = 0; i < 8; i++) {
    tx_msg.buffer[i] = message.data[i];
  }

  const auto tx_result = twai_node_transmit(node_hdl, &tx_msg, 0);
  if (count % DEBUG_PRINT_INTERVAL == 0) {
#ifdef CAN_DEBUG
    if (tx_result == ESP_ERR_TIMEOUT) {
      Serial.println("Transmit Fail: ESP_ERR_TIMEOUT");
      return;
    }
    if (tx_result == ESP_ERR_INVALID_ARG) {
      Serial.println("Transmit Fail: ESP_ERR_INVALID_ARG");
      return;
    }
    if (tx_result == ESP_ERR_INVALID_STATE) {
      Serial.println("Transmit Fail: ESP_ERR_INVALID_STATE");
      return;
    }
    if (tx_result == ESP_FAIL) {
      Serial.println("Transmit Fail: ESP_FAIL");
      return;
    }
    if (tx_result == ESP_ERR_NOT_SUPPORTED) {
      Serial.println("Transmit Fail: ESP_ERR_NOT_SUPPORTED");
      return;
    }
    if (tx_result != ESP_OK) {
      Serial.println("Transmit Fail: Unexpected error: " + String(tx_result));
      return;
    }
#endif
  }

  if (twai_node_transmit_wait_all_done(node_hdl, 100) != ESP_OK) {
#ifdef CAN_DEBUG
    Serial.println("Transmit Fail: Waiting for transmission completion failed");
#endif
    return;
  }
}

void CanCommunicator::receive() const {
  // Reception is handled by the TWAI ISR callback.
  // This method is provided to satisfy the CanReceiver interface.
}

bool CanCommunicator::receive(twai_node_handle_t handle,
                              const twai_rx_done_event_data_t* edata,
                              void* user_ctx) {
  static uint32_t count = 0;
  auto* communicator = static_cast<CanCommunicator*>(user_ctx);

  uint8_t recv_buff[8];
  twai_frame_t rx_frame = {
      .buffer = recv_buff,
      .buffer_len = sizeof(recv_buff),
  };

  const auto rx_result =
      twai_node_receive_from_isr(communicator->node_hdl, &rx_frame);
  if (rx_result != ESP_OK) {
    Serial.println("Receive fail: twai_node_receive_from_isr error");
    return false;
  }

  if (count % DEBUG_PRINT_INTERVAL == 0) {
#ifdef CAN_DEBUG
    if (rx_result == ESP_ERR_TIMEOUT) {
      Serial.println("Receive fail: ESP_ERR_TIMEOUT");
      return false;
    }
    if (rx_result == ESP_ERR_INVALID_ARG) {
      Serial.println("Receive fail: ESP_ERR_INVALID_ARG");
      return false;
    }
    if (rx_result == ESP_ERR_INVALID_STATE) {
      Serial.println("Receive fail: ESP_ERR_INVALID_STATE");
      return false;
    }
    if (rx_result != ESP_OK) {
      Serial.println("Receive fail: Unexpected error: " + String(rx_result));
      return false;
    }
    if (rx_frame.header.rtr) {
      Serial.println("Receive Fail: The received message is a remote frame!");
      return false;
    }
    if (rx_frame.header.ide) {
      Serial.println(
          "Receive Fail: The received message is an extended frame!");
      return false;
    }
#endif
  }

  const auto rx_id = rx_frame.header.id;
  std::array<uint8_t, 8> rx_buf = {};
  for (uint8_t i = 0; i < 8; i++) {
    rx_buf[i] = rx_frame.buffer[i];
  }

  for (const auto& listener : communicator->receive_event_listeners) {
    if (std::any_of(
            listener.first.begin(), listener.first.end(),
            [&rx_id](const can::CanId& can_id) { return can_id == rx_id; })) {
      listener.second(rx_id, rx_buf);
    }
  }

  return false;
}

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
        listener) {
  receive_event_listeners.push_back(
      std::make_pair(listening_can_ids, listener));
}
}  // namespace can