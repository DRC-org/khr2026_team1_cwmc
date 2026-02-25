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

  // BUS_OFF / BUS_RECOVERED / ERR_PASS / ERR_ACTIVE をアラートで検知
  g_config_.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                             TWAI_ALERT_ERR_PASS | TWAI_ALERT_ERR_ACTIVE;

  t_config_ = TWAI_TIMING_CONFIG_1MBITS();
  // 3点サンプリングでビットエラー耐性を向上（REC 蓄積の抑制）
  t_config_.triple_sampling = true;
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

  if (alerts & TWAI_ALERT_BUS_OFF) {
    bus_off_ = true;
    Serial.println("TWAI: BUS_OFF, リカバリ開始...");
    // CAN 規定の 128×11 bit リセッシブ列を待つ正規の復帰手順（≒1.4ms）。
    // uninstall/install は GPIO を再設定してバス上にグリッチを起こすうえ、
    // 100ms クールダウン中に ACK が返らず他ノードの TEC
    // が蓄積するため使わない。
    twai_initiate_recovery();
  } else if (alerts & TWAI_ALERT_ERR_PASS) {
    // 初回遷移時のみログ出力。繰り返しログは UART バッファを埋めて
    // ControlTask をブロッキングさせるため、状態フラグで抑制する。
    if (!err_passive_) {
      err_passive_ = true;
      twai_status_info_t status;
      twai_get_status_info(&status);
      Serial.printf("TWAI: Error Passive (TEC=%lu, REC=%lu, overrun=%lu)\n",
                    status.tx_error_counter, status.rx_error_counter,
                    status.rx_overrun_count);

      // TEC が高い場合のみリカバリ: 送信エラーが BUS_OFF に至るのを防ぐ。
      // REC が高い場合は受信ノイズが原因であり、stop/start
      // しても即座に戻るだけ。
      if (status.tx_error_counter > 100) {
        if (twai_stop() == ESP_OK) {
          twai_start();
        }
      }
    }
  }

  if (alerts & TWAI_ALERT_ERR_ACTIVE) {
    // Error Passive から Error Active に回復したとき（REC/TEC < 128）
    if (err_passive_) {
      err_passive_ = false;
      Serial.println("TWAI: Error Active に回復");
    }
  }

  if (alerts & TWAI_ALERT_BUS_RECOVERED) {
    // リカバリ完了後は Stopped 状態になるので twai_start() で再開
    twai_start();
    bus_off_ = false;
    Serial.println("TWAI: リカバリ完了");
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

  // タイムアウト 0 で非ブロッキング送信。
  // キュー詰まり時は即座に破棄し、3ms 制御周期のオーバランを防ぐ。
  twai_transmit(&tx_msg, 0);
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
