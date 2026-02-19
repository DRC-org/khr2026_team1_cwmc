#include "can_communicator.hpp"

#include <Arduino.h>

#ifndef CAN_TX
#define CAN_TX GPIO_NUM_16
#endif
#ifndef CAN_RX
#define CAN_RX GPIO_NUM_4
#endif

namespace can {

CanCommunicator::CanCommunicator() {}

void CanCommunicator::setup(twai_filter_config_t filter_config) {
  g_config_ = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX,
                                          (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  g_config_.tx_queue_len = 10;
  g_config_.rx_queue_len = 50;

  // BUS_OFF / BUS_RECOVERED / ERR_PASS をアラートで検知
  g_config_.alerts_enabled =
      TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS;

  t_config_ = TWAI_TIMING_CONFIG_1MBITS();
  f_config_ = filter_config;

  if (twai_driver_install(&g_config_, &t_config_, &f_config_) == ESP_OK) {
    Serial.println("TWAI Driver installed");
  } else {
    Serial.println("Failed to install TWAI driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver started");
  } else {
    Serial.println("Failed to start TWAI driver");
  }
}

void CanCommunicator::handle_alerts() {
  // bus_off_ 中でもアラートを読み続け BUS_RECOVERED を検知する
  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) != ESP_OK) {
    return;
  }

  if (alerts & TWAI_ALERT_ERR_PASS) {
    // エラーパッシブ状態（TEC/REC > 127）: BUS_OFF 予兆として記録
    Serial.println("TWAI: Error Passive state");
  }

  if (alerts & TWAI_ALERT_BUS_OFF) {
    bus_off_ = true;
    Serial.println("TWAI BUS_OFF, リカバリ開始...");
    // CAN 規定の 128×11 bit リセッシブ列を待つ正規の復帰手順（≒1.4ms）。
    // uninstall/install は GPIO を再設定してバス上にグリッチを起こすうえ、
    // 100ms クールダウン中に ACK が返らず他ノードの TEC が蓄積するため使わない。
    twai_initiate_recovery();
  }

  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    // リカバリ完了後は Stopped 状態になるので twai_start() で再開
    twai_start();
    bus_off_ = false;
    Serial.println("TWAI: リカバリ完了、再開");
  }
}

void CanCommunicator::transmit(const CanTxMessage message) {
  // BUS_OFF 復帰中は送信しない
  if (bus_off_) {
    return;
  }

  twai_message_t tx_msg;
  tx_msg.identifier = message.id;
  tx_msg.extd = 0;  // 標準フレーム (11-bit ID)
  tx_msg.rtr = 0;   // データフレーム
  tx_msg.ss = 1;    // シングルショット（再送しない）
  tx_msg.self = 0;  // セルフ受信なし
  tx_msg.data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    tx_msg.data[i] = message.data[i];
  }

  twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
}

void CanCommunicator::process_received_messages() {
  handle_alerts();

  if (bus_off_) {
    return;
  }

  twai_message_t rx_msg;

  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (!rx_msg.rtr) {
      std::array<uint8_t, 8> data_array;
      for (int i = 0; i < 8; i++) {
        data_array[i] = rx_msg.data[i];
      }

      for (const auto& listener_pair : receive_event_listeners) {
        const auto& target_ids = listener_pair.first;
        const auto& callback = listener_pair.second;

        // ID リストが空なら全受信、指定があれば一致確認
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
}

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
        listener) {
  receive_event_listeners.push_back({listening_can_ids, listener});
}

}  // namespace can
