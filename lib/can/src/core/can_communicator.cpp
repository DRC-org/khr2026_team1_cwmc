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
      // NO_ACK モード: ACK ビットを要求しないため ACK エラーが発生せず TEC が上昇しない。
      // これによりモーター側の瞬断でもバスオフを防げる。受信は正常に動作する。
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NO_ACK);
  general_config.tx_queue_len = 0;
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();

  const auto driver_install_result = twai_driver_install_v2(
      &general_config, &timing_config, &filter_config, &twai_handle);
  if (driver_install_result == ESP_OK) {
    Serial.println("Driver install OK!");
  } else {
    Serial.println("Driver install fail!");
    return;
  }

  const auto start_result = twai_start_v2(twai_handle);
  if (start_result == ESP_OK) {
    Serial.println("Driver start OK!");
  } else {
    Serial.println("Driver start fail!");
    return;
  }
}

void CanCommunicator::check_errors() const {
  twai_status_info_t status;
  if (twai_get_status_info_v2(twai_handle, &status) != ESP_OK) return;

  if (status.state == TWAI_STATE_RUNNING && status.tx_error_counter > 96) {
    // TEC がエラーパッシブ閾値 (127) に達する前にドライバを再起動して TEC をリセット。
    // twai_stop → twai_start はハードウェアリセットモードを経由するため TEC が 0 に戻る。
    // バスオフを未然に防ぐことで ss=1 + ERRATA_FIX_TX_INTR_LOST の assertion バグを回避。
    twai_stop_v2(twai_handle);
    twai_start_v2(twai_handle);
#ifdef CAN_DEBUG
    Serial.println("CAN: restarted (TEC=" + String(status.tx_error_counter) + ")");
#endif
  } else if (status.state == TWAI_STATE_BUS_OFF) {
    // フォールバック: TWAI_MODE_NO_ACK とプリエンプティブ再起動で通常は到達しないはず
    twai_initiate_recovery_v2(twai_handle);
    vTaskDelay(pdMS_TO_TICKS(10));
    twai_start_v2(twai_handle);
#ifdef CAN_DEBUG
    Serial.println("CAN: bus-off recovered");
#endif
  }
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

  const auto tx_result = twai_transmit_v2(twai_handle, &tx_message, 0);

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

  twai_message_t rx_message;
  const auto rx_result = twai_receive_v2(twai_handle, &rx_message, 0);

  // 150ms ごとに TEC を確認し、バスオフに達する前にプリエンプティブ再起動する
  if (count % 50 == 0) {
    check_errors();
  }

  if (rx_result == ESP_ERR_INVALID_STATE) {
    check_errors();
    return;
  }

  // ESP_ERR_TIMEOUT はキューが空の正常状態 (timeout=0 のため頻繁に発生)
  if (rx_result != ESP_OK) {
    return;
  }

  if (rx_message.rtr || rx_message.extd) {
    return;
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

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
        listener) {
  receive_event_listeners.push_back(
      std::make_pair(listening_can_ids, listener));
}
}  // namespace can
