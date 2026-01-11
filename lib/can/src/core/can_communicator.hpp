#pragma once

#include <Arduino.h>
#include <esp_twai.h>
#include <esp_twai_onchip.h>

#include <vector>

#include "../interfaces/can_receiver.hpp"
#include "../interfaces/can_transmitter.hpp"

namespace can {
using CanId = uint32_t;

/// @brief CAN通信を行うクラス
/// @details
///     CAN通信を行うクラスです。
///     CAN通信の送信と受信を行うためのCanTransmitterとCanReceiverを継承しています。
///     使う前にはsetup()関数を呼び出して初期化する必要があります。
///     複数のインスタンスを生成して使うことはできません…
///     (esp32が複数のCANコントローラをサポートしていないため)
/// @see
/// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html
class CanCommunicator : public CanTransmitter, public CanReceiver {
 public:
  CanCommunicator();

  /// @brief セットアップ処理。使う前に呼び出す！
  void setup(
      twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL());
  void transmit(const CanTxMessage message) const override;
  static bool receive(twai_node_handle_t handle,
                      const twai_rx_done_event_data_t* edata, void* user_ctx);
  void add_receive_event_listener(
      std::vector<can::CanId> listening_can_ids,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
          listener) override;

 private:
  /// @brief CAN通信インスタンス
  twai_node_handle_t node_hdl;

  /// @brief CAN受信時のイベントリスナのリスト
  std::vector<std::pair<
      std::vector<can::CanId>,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>>>
      receive_event_listeners;
};
}  // namespace can