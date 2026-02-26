#include "can_communicator.hpp"

#include <Arduino.h>

namespace can {
constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

CanCommunicator::CanCommunicator()
    : node_hdl(NULL), receive_event_listeners() {}

// TODO: エラーからの復帰

void CanCommunicator::setup(twai_mask_filter_config_t filter_config) {
  twai_onchip_node_config_t node_config = {};
  node_config.io_cfg.tx = CAN_TX;
  node_config.io_cfg.rx = CAN_RX;
  node_config.bit_timing.bitrate = 1000000;  // 1Mbps
  node_config.tx_queue_depth = 20;
  node_config.flags.no_receive_rtr = true;

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

  // フィルタ設定（デフォルトは全受信）
  twai_mask_filter_config_t default_config = {};
  if (memcmp(&filter_config, &default_config,
             sizeof(twai_mask_filter_config_t)) != 0) {
    if (twai_node_config_mask_filter(node_hdl, 0, &filter_config) != ESP_OK) {
      Serial.println("Failed to set custom filter config");
    } else {
      Serial.println("Custom filter config applied");
    }
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
  uint8_t send_buff[8] = {0};

  twai_frame_t tx_msg = {};
  tx_msg.header.id = message.id;
  tx_msg.header.ide = false;  // 標準フレーム (11-bit ID)
  tx_msg.header.fdf = false;  // CAN FD は使わない
  tx_msg.header.dlc = 8;

  for (int i = 0; i < 8; i++) {
    send_buff[i] = message.data[i];
  }

  tx_msg.buffer = send_buff;
  tx_msg.buffer_len = sizeof(send_buff);

  // ControlTask の 3ms 周期を守るためノンブロッキング送信
  if (twai_node_transmit(node_hdl, &tx_msg, 0) == ESP_OK) {
    return;
  }
}

bool CanCommunicator::receive(twai_node_handle_t handle,
                              const twai_rx_done_event_data_t* edata,
                              void* user_ctx) {
  auto* communicator = static_cast<CanCommunicator*>(user_ctx);

  uint8_t recv_buff[8];
  twai_frame_t rx_frame = {
      .buffer = recv_buff,
      .buffer_len = sizeof(recv_buff),
  };

  const auto rx_result =
      twai_node_receive_from_isr(communicator->node_hdl, &rx_frame);
  if (rx_result != ESP_OK) {
#ifdef CAN_DEBUG
    Serial.println("Receive fail: twai_node_receive_from_isr error");
#endif
    return false;
  }

  std::array<uint8_t, 8> data_array;
  for (uint8_t i = 0; i < rx_frame.header.dlc; i++) {
    data_array[i] = rx_frame.buffer[i];
  }

  // 登録されたリスナーに通知
  for (const auto& listener_pair : communicator->receive_event_listeners) {
    const auto& target_ids = listener_pair.first;
    const auto& callback = listener_pair.second;

    // ターゲットIDリストが空なら「すべて受信」、指定があれば一致確認
    bool match = target_ids.empty();
    if (!match) {
      for (const auto& id : target_ids) {
        if (id == rx_frame.header.id) {
          match = true;
          break;
        }
      }
    }

    // マッチしたらコールバック実行
    if (match && callback) {
      callback(rx_frame.header.id, data_array);
    }
  }

  return true;
}

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
        listener) {
  // リスナーリストに追加
  receive_event_listeners.push_back({listening_can_ids, listener});
}

}  // namespace can
