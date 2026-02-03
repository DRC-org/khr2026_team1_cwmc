#include "can_communicator.hpp"
#include <Arduino.h>

// 【重要】お使いのボードに合わせてピン番号を変更してください
// 一般的なESP32でのCANピン設定例 (GPIO 5, 4)
#ifndef CAN_TX
#define CAN_TX GPIO_NUM_5
#endif
#ifndef CAN_RX
#define CAN_RX GPIO_NUM_4
#endif

namespace can {

CanCommunicator::CanCommunicator() {
    // コンストラクタでは特に行うことはありません
}

void CanCommunicator::setup(twai_filter_config_t filter_config) {
    // 1. 一般設定 (TXピン, RXピン, モード設定)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    
    // 2. タイミング設定 (500kbit/s)
    // 通信相手のビットレートと必ず合わせる必要があります
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // 3. フィルタ設定 (引数から受け取る)
    twai_filter_config_t f_config = filter_config;

    // ドライバのインストール
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI Driver installed");
    } else {
        Serial.println("Failed to install TWAI driver");
        return;
    }

    // ドライバの開始
    if (twai_start() == ESP_OK) {
        Serial.println("TWAI Driver started");
    } else {
        Serial.println("Failed to start TWAI driver");
    }
}

void CanCommunicator::transmit(const CanTxMessage message) const {
    // 送信メッセージの構築
    twai_message_t tx_msg;
    tx_msg.identifier = message.id;
    tx_msg.extd = 0;              // 標準フレーム (11-bit ID)
    tx_msg.rtr = 0;               // データフレーム
    tx_msg.ss = 0;                // シングルショット送信無効（再送あり）
    tx_msg.self = 0;              // 自分自身には送信しない
    tx_msg.data_length_code = 8;  // 常に8バイト送信（必要に応じて変更可）

    // データのコピー
    for (int i = 0; i < 8; i++) {
        tx_msg.data[i] = message.data[i];
    }

    // 送信キューに入れる (タイムアウト100ms)
    // 送信成功ではなく「キューに入ったか」を確認しています
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) != ESP_OK) {
        Serial.println("Failed to queue message for transmission");
    }
}

void CanCommunicator::process_received_messages() {
    twai_message_t rx_msg;

    // 受信キューにメッセージがあるか確認 (待機時間0でポーリング)
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        
        // データフレームのみ処理 (RTRフレームは無視)
        if (!rx_msg.rtr) {
            
            // std::array に変換
            std::array<uint8_t, 8> data_array;
            for(int i=0; i<8; i++) {
                data_array[i] = rx_msg.data[i];
            }

            // 登録されたリスナーに通知
            for (const auto& listener_pair : receive_event_listeners) {
                const auto& target_ids = listener_pair.first;
                const auto& callback = listener_pair.second;

                // ターゲットIDリストが空なら「すべて受信」、指定があれば一致確認
                bool match = target_ids.empty();
                if (!match) {
                    for (const auto& id : target_ids) {
                        if (id == rx_msg.identifier) {
                            match = true;
                            break;
                        }
                    }
                }

                // マッチしたらコールバック実行
                if (match && callback) {
                    callback(rx_msg.identifier, data_array);
                }
            }
        }
    }
}

void CanCommunicator::add_receive_event_listener(
    std::vector<can::CanId> listening_can_ids,
    std::function<void(const can::CanId, const std::array<uint8_t, 8>)> listener) {
    
    // リスナーリストに追加
    receive_event_listeners.push_back({listening_can_ids, listener});
}

}  // namespace can