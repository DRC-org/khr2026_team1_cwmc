#pragma once

#include <Arduino.h>
#include <driver/twai.h>

#include <array>
#include <functional>
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
  void setup(twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL());

  /// @brief メッセージ送信
  void transmit(const CanTxMessage message) const override;

  /// @brief RX キューを処理してリスナーを呼び出す。3ms ごとに呼ぶ。
  void receive() override;

  /// @brief イベントリスナ登録
  void add_receive_event_listener(
      std::vector<can::CanId> listening_can_ids,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>
          listener) override;

 private:
  /// @brief CAN受信時のイベントリスナのリスト
  std::vector<std::pair<
      std::vector<can::CanId>,
      std::function<void(const can::CanId, const std::array<uint8_t, 8>)>>>
      receive_event_listeners;
};
}  // namespace can
