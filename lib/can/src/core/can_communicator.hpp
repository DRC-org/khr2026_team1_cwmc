#pragma once

#include <Arduino.h>

#include <array>       // std::arrayのために必要
#include <functional>  // std::functionのために必要
#include <vector>

#include "../interfaces/can_receiver.hpp"
#include "../interfaces/can_transmitter.hpp"
#include "driver/twai.h"

namespace can {
using CanId = uint32_t;

/// @brief CAN通信を行うクラス
class CanCommunicator : public CanTransmitter, public CanReceiver {
 public:
  CanCommunicator();

  /// @brief セットアップ処理
  /// @param filter_config フィルタ設定（デフォルトは全受信）
  void setup(
      twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL());

  /// @brief メッセージ送信
  void transmit(const CanTxMessage message) override;

  /// @brief 受信処理（loop内で呼ぶ）
  void process_received_messages() override;

  /// @brief イベントリスナ登録
  void add_receive_event_listener(
      std::vector<can::CanId> listening_can_ids,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
          listener) override;

 private:
  void handle_alerts();

  twai_general_config_t g_config_;
  twai_timing_config_t t_config_;
  twai_filter_config_t f_config_;

  volatile bool bus_off_ = false;
  TickType_t restart_tick_ = 0;

  /// @brief CAN受信時のイベントリスナのリスト
  std::vector<std::pair<
      std::vector<can::CanId>,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>>>
      receive_event_listeners;
};
}  // namespace can
