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
    // 1. 一般設定 (TXピン, RXピン, モード設定)
    // バスオフ状態からの自動復帰を有効にするため TWAI_ALERT_BUS_RECOVERED を監視するか、
    // あるいは単純にドライバの自動復帰機能を信頼して運用します。
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 50;
    g_config.rx_queue_len = 50;
    // エラー時に自動でバスオフから復帰するように設定（重要）
    // これを設定しないと、エラーが蓄積してバスオフになった際、手動でリセットしない限り復帰できません
    // ただし、ESP-IDFのバージョンによっては構造体のメンバ名が異なる場合があるため注意が必要ですが、
    // Arduino-ESP32 では通常デフォルトで自動復帰は無効です。
    // ここではドライバ再起動を試みるロジックを別途入れるか、またはアラートを監視するのが定石です。
    
    // 2. タイミング設定 (1Mbit/s)
    // 通信相手のビットレートと必ず合わせる必要があります
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

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
    if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) != ESP_OK) {
        // エラー詳細を表示
        uint32_t alerts = 0;
        twai_read_alerts(&alerts, 0);
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        
        Serial.print("Failed to queue message. State: ");
        Serial.print(status_info.state);
        Serial.print(", TED: ");
        Serial.print(status_info.tx_error_counter);
        Serial.print(", RED: ");
        Serial.print(status_info.rx_error_counter);
        Serial.print(", Alerts: ");
        Serial.println(alerts, HEX);

        // バスオフ状態なら復旧を試みる（簡易的な処置）
        if (status_info.state == TWAI_STATE_BUS_OFF) {
            Serial.println("Bus Off detected. Initiating recovery...");
            twai_initiate_recovery();
        }
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

            // 【デバッグ用】全受信メッセージをログ出力
            // Serial.print("CAN RX ID: 0x");
            // Serial.print(rx_msg.identifier, HEX);
            // Serial.print(" Data: ");
            // for(int i=0; i<8; i++) {
            //     Serial.print(data_array[i], HEX);
            //     Serial.print(" ");
            // }
            // Serial.println();

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