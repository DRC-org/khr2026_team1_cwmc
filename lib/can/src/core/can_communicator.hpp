#pragma once

#include <Arduino.h>
#include <esp_twai.h>
#include <esp_twai_onchip.h>
#include <freertos/queue.h>

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "../interfaces/can_receiver.hpp"
#include "../interfaces/can_transmitter.hpp"

namespace can {
using CanId = uint32_t;

/// @brief CAN通信を行うクラス
class CanCommunicator : public CanTransmitter, public CanReceiver {
 public:
  CanCommunicator();

  /// @brief セットアップ処理
  /// @param filter_config フィルタ設定（デフォルトは全受信）
  void setup(twai_mask_filter_config_t filter_config = {});

  /// @brief メッセージ送信
  void transmit(const CanTxMessage message) const override;

  /// @brief イベントリスナ登録
  void add_receive_event_listener(
      std::vector<can::CanId> listening_can_ids,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
          listener) override;

 private:
  /// @brief CAN 通信のインスタンス
  twai_node_handle_t node_hdl;

  static bool receive(twai_node_handle_t handle,
                      const twai_rx_done_event_data_t* edata, void* user_ctx);

  /// @brief CAN受信時のイベントリスナのリスト
  std::vector<std::pair<
      std::vector<can::CanId>,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>>>
      receive_event_listeners;
};
}  // namespace can
