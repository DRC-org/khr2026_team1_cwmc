#include "can_communicator.hpp"

#define CAN_DEBUG

namespace can {
/// @brief デバッグ出力の間隔(ループ回数)
constexpr uint8_t DEBUG_PRINT_INTERVAL = 1;

constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
constexpr gpio_num_t CAN_RX = GPIO_NUM_4;

CanCommunicator::CanCommunicator() : receive_event_listeners() {}

void CanCommunicator::setup(twai_filter_config_t filter_config) {
  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 0;
  // 4モーター × フィードバック頻度 × 制御周期 3ms
  // 分のバーストを収容できるサイズ
  general_config.rx_queue_len = 20;
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();

  const auto driver_install_result =
      twai_driver_install(&general_config, &timing_config, &filter_config);
  if (driver_install_result == ESP_OK) {
    Serial.println("Driver install OK!");
  } else {
    Serial.println("Driver install fail!");
    return;
  }

  const auto start_result = twai_start();
  if (start_result == ESP_OK) {
    Serial.println("Driver start OK!");
  } else {
    Serial.println("Driver start fail!");
    return;
  }
}

void CanCommunicator::check_errors() const {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (status.state == TWAI_STATE_BUS_OFF) {
    // twai_initiate_recovery() は ss=1 + CONFIG_TWAI_ERRATA_FIX_TX_INTR_LOST と
    // 組み合わせると assert クラッシュを引き起こす。
    // twai_stop() は BUS_OFF 状態でも呼び出し可能。
    // stop → start はハードウェアリセット経由のため TEC と REC が両方 0 にリセットされる。
    twai_stop();
    twai_start();
#ifdef CAN_DEBUG
    Serial.println("CAN: bus-off recovered");
#endif
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
#ifdef CAN_DEBUG
    Serial.println("CAN: restarted from STOPPED");
#endif
  }
  // TEC>96 によるプリエンプティブ再起動を廃止。
  // twai_stop() 呼び出しが C620 フレームの受信を中断し ESP32 の REC を蓄積させ、
  // エラーパッシブ状態に陥って C620 フィードバックが無音で破棄される根本原因だった。
  // ss=1 + ERRATA_FIX_TX_INTR_LOST の TX 割り込み消失 (Transmit Fail: -1) も同原因。
}

void CanCommunicator::transmit(const CanTxMessage message) const {
  static uint32_t count = 0;
  count++;

  twai_message_t tx_message;
  tx_message.identifier = message.id;
  tx_message.extd = 0;
  tx_message.rtr = 0;
  tx_message.ss = 1;
  tx_message.self = 0;
  tx_message.dlc_non_comp = 0;
  tx_message.data_length_code = 8;
  for (uint8_t i = 0; i < 8; i++) {
    tx_message.data[i] = message.data[i];
  }

  const auto tx_result = twai_transmit(&tx_message, 0);

  if (tx_result == ESP_ERR_INVALID_STATE) {
    check_errors();
    return;
  }

#ifdef CAN_DEBUG
  if (count % DEBUG_PRINT_INTERVAL == 0 && tx_result != ESP_OK) {
    Serial.print("Transmit Fail: ");
    Serial.println(tx_result);
  }
#endif
}

void CanCommunicator::receive() const {
  static uint32_t count = 0;
  count++;

  if (count % 10 == 0) {
    check_errors();
  }

  // キュー内の全メッセージを処理する。
  // 1 呼び出しで 1 メッセージしか処理しないと、4 モーターからのフィードバックが
  // キューに溜まりオーバーフローで破棄され、current_rpm が更新されなくなる。
  while (true) {
    twai_message_t rx_message;
    const auto rx_result = twai_receive(&rx_message, 0);

    if (rx_result == ESP_ERR_INVALID_STATE) {
      check_errors();
      break;
    }

    // ESP_ERR_TIMEOUT はキューが空の正常状態
    if (rx_result != ESP_OK) {
      break;
    }

    if (rx_message.rtr || rx_message.extd) {
      continue;
    }

    const auto rx_id = rx_message.identifier;
    std::array<uint8_t, 8> rx_buf = {};
    for (uint8_t i = 0; i < 8; i++) {
      rx_buf[i] = rx_message.data[i];
    }

    for (const auto& listener : receive_event_listeners) {
      if (std::any_of(
              listener.first.begin(), listener.first.end(),
              [&rx_id](const can::CanId& can_id) { return can_id == rx_id; })) {
        listener.second(rx_id, rx_buf);
      }
    }
  }
}

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
        listener) {
  receive_event_listeners.push_back(
      std::make_pair(listening_can_ids, listener));
}
}  // namespace can
