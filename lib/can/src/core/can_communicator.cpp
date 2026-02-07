#include "can_communicator.hpp"
#include <Arduino.h>

// 【重要】お使いのボードに合わせてピン番号を変更してください
// 一般的なESP32でのCANピン設定例 (GPIO 5, 4)
#ifndef CAN_TX
#define CAN_TX GPIO_NUM_16
#endif
#ifndef CAN_RX
#define CAN_RX GPIO_NUM_4
#endif

namespace can {

CanCommunicator::CanCommunicator() {
    // コンストラクタでは特に行うことはありません
}

void CanCommunicator::setup(twai_filter_config_t filter_config) {
    g_config_ = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    g_config_.tx_queue_len = 10;
    g_config_.rx_queue_len = 50;
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

void CanCommunicator::restart_driver() {
    Serial.println("TWAI restarting driver...");
    twai_stop();              // 失敗しても続行（BUS_OFFでは失敗する）
    twai_driver_uninstall();  // 内部状態を完全にクリア
    twai_driver_install(&g_config_, &t_config_, &f_config_);
    twai_start();
    Serial.println("TWAI driver restarted");
}

void CanCommunicator::transmit(const CanTxMessage message) {
    // 送信メッセージの構築
    twai_message_t tx_msg;
    tx_msg.identifier = message.id;
    tx_msg.extd = 0;              // 標準フレーム (11-bit ID)
    tx_msg.rtr = 0;               // データフレーム
    tx_msg.ss = 0;                // 自動再送（送信失敗時にリトライ）
    tx_msg.self = 0;              // 自分自身には送信しない
    tx_msg.data_length_code = 8;

    // データのコピー
    for (int i = 0; i < 8; i++) {
        tx_msg.data[i] = message.data[i];
    }

    // 短いタイムアウトで送信（1フレーム送信に十分な時間）
    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
    if (err != ESP_OK) {
        // Bus Off の場合のみドライバを再起動
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            restart_driver();
        }
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