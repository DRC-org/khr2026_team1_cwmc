#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <stdlib.h>

#include <can/core.hpp>
#include <can/peripheral.hpp>

// タスクハンドル定義
TaskHandle_t MicroROSTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;
SemaphoreHandle_t DataMutex = NULL;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_timer_t timer;
std_msgs__msg__String send_msg;
std_msgs__msg__String recv_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

can::CanCommunicator* can_comm;

volatile int16_t current_rpm_fl = 0;
volatile int16_t current_rpm_fr = 0;
volatile int16_t current_rpm_rl = 0;
volatile int16_t current_rpm_rr = 0;

volatile int16_t target_rpms[4] = {0, 0, 0, 0};

// フィードバック用の制御値
volatile int32_t output_currents_fb[4] = {0, 0, 0, 0};
volatile float p_terms_fb[4] = {0, 0, 0, 0};
volatile float i_terms_fb[4] = {0, 0, 0, 0};
volatile float d_terms_fb[4] = {0, 0, 0, 0};

// PID 制御用（ランタイムで変更可能）
volatile float pid_kp = 0.5f;
volatile float pid_ki = 0.05f;
volatile float pid_kd = 0.0f;
#define DT 0.003f  // 制御周期 [秒] (3ms)

#define CLAMPING_OUTPUT \
  5000  // 電流値のクランピング値 [mA] (M3508: Max 16384 -> 20A)
#define INTEGRAL_LIMIT 10000.0f  // 積分項の上限値（Anti-windup用）
// #define STOP_THRESHOLD_RPM 50.0f  // 停止判定の閾値 [RPM]

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }

#define RCSOFTCHECK(fn)             \
  {                                 \
    rcl_ret_t temp_rc = fn;         \
    if ((temp_rc != RCL_RET_OK)) {  \
      Serial.print("Soft error: "); \
      Serial.println(temp_rc);      \
    }                               \
  }

// TODO: エラー発生時の対処は何とかする
void error_loop() {
  while (1) {
    Serial.println("An error occurred in micro-ROS!");
    delay(5000);  // 5秒待機に変更
  }
}

// rspi と Wi-Fi 経由で接続する場合用
#if (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
char ssid[] = "DRC";
char psk[] = "28228455";
IPAddress agent_ip(192, 168, 0, 101);
size_t agent_port = 8888;
#endif

void register_can_event_handlers() {
  // M3508 のフィードバック値を受け取る
  can_comm->add_receive_event_listener(
      {0x201, 0x202, 0x203, 0x204},
      [&](const can::CanId id, const std::array<uint8_t, 8> data) {
        int16_t rpm = (data[2] << 8) | data[3];

        // Mutexで保護して書き込み
        if (xSemaphoreTake(DataMutex, 0) == pdTRUE) {
          switch (id) {
            case 0x201:
              current_rpm_fl = rpm;
              break;
            case 0x202:
              current_rpm_fr = rpm;
              break;
            case 0x203:
              current_rpm_rl = rpm;
              break;
            case 0x204:
              current_rpm_rr = rpm;
              break;
            default:
              break;
          }
          xSemaphoreGive(DataMutex);
        }
      });
}

// PID 制御
void compute_motor_commands(int16_t target_rpm[4], int32_t out_current[4]) {
  static float prev_rpm_errors[4] = {0, 0, 0, 0};
  static float integral_errors[4] = {0, 0, 0, 0};

  // Mutexで保護して読み込み
  int16_t current_rpms[4];
  float local_kp, local_ki, local_kd;
  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    current_rpms[0] = current_rpm_fl;
    current_rpms[1] = current_rpm_fr;
    current_rpms[2] = current_rpm_rl;
    current_rpms[3] = current_rpm_rr;
    // PIDゲインもコピー
    local_kp = pid_kp;
    local_ki = pid_ki;
    local_kd = pid_kd;
    xSemaphoreGive(DataMutex);
  } else {
    // 取得失敗時は前回値か0を使うなどの対策が必要だが、portMAX_DELAYなので基本ここには来ない
    return;
  }

  float rpm_errors[4];
  rpm_errors[0] = static_cast<float>(target_rpm[0]) - current_rpms[0];
  rpm_errors[1] = static_cast<float>(target_rpm[1]) - current_rpms[1];
  rpm_errors[2] = static_cast<float>(target_rpm[2]) - current_rpms[2];
  rpm_errors[3] = static_cast<float>(target_rpm[3]) - current_rpms[3];

  for (uint8_t i = 0; i < 4; i++) {
    float error = rpm_errors[i];

    // 微分項（正しい微分：差分をdtで割る）
    float d_error = (error - prev_rpm_errors[i]) / DT;
    prev_rpm_errors[i] = error;

    int32_t command_current = 0;

    // 目標 RPM が 0 のときのハンチング防止：閾値以下なら出力を 0 にする
    /* if (target_rpm[i] == 0 && fabs(error) < STOP_THRESHOLD_RPM) {
      command_current = 0;
      prev_rpm_errors[i] = 0.0f;  // 微分項の蓄積もリセット
      integral_errors[i] = 0.0f;  // 積分項もリセット
    } else { */

    // PID 制御の計算（仮の出力）
    float p_term = local_kp * error;
    float i_term = local_ki * integral_errors[i];
    float d_term = local_kd * d_error;
    float pid_output = p_term + i_term + d_term;

    command_current = static_cast<int32_t>(pid_output);

    // クランピング前の値を保存
    int32_t unclamped_current = command_current;

    // クランピング
    if (command_current > CLAMPING_OUTPUT) {
      command_current = CLAMPING_OUTPUT;
    } else if (command_current < -CLAMPING_OUTPUT) {
      command_current = -CLAMPING_OUTPUT;
    }

    // Anti-windup: 出力が飽和していない場合のみ積分を更新
    if (command_current == unclamped_current) {
      // 飽和していない場合は積分を更新
      integral_errors[i] += error * DT;

      // 積分項自体にも上限を設定
      if (integral_errors[i] > INTEGRAL_LIMIT) {
        integral_errors[i] = INTEGRAL_LIMIT;
      } else if (integral_errors[i] < -INTEGRAL_LIMIT) {
        integral_errors[i] = -INTEGRAL_LIMIT;
      }
    }
    // 飽和している場合は積分を更新しない（Anti-windup）
    // }

    out_current[i] = command_current;

    // フィードバック用に値を保存
    if (xSemaphoreTake(DataMutex, 0) == pdTRUE) {
      output_currents_fb[i] = command_current;
      p_terms_fb[i] = p_term;
      i_terms_fb[i] = i_term;
      d_terms_fb[i] = d_term;
      xSemaphoreGive(DataMutex);
    }
  }
}

// C620 用、電流値を CAN 通信用のバイト列に変換する
void milli_amperes_to_bytes(const int32_t milli_amperes[4],
                            uint8_t out_tx_buf[8]) {
  for (uint8_t i = 0; i < 4; i++) {
    int32_t milli_ampere = milli_amperes[i] * 16384 / 20000;
    uint8_t upper = (milli_ampere >> 8) & 0xFF;
    uint8_t lower = milli_ampere & 0xFF;
    out_tx_buf[i * 2] = upper;
    out_tx_buf[i * 2 + 1] = lower;
  }
}

// rspi からの JSON を受け取る
IRAM_ATTR void subscription_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;

  // JSON をパース
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, msg->data.data);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  // m3508_rpms コマンドの処理
  if (doc.containsKey("m3508_rpms")) {
    float fl_rpm = doc["m3508_rpms"]["fl"] | 0.0;
    float fr_rpm = doc["m3508_rpms"]["fr"] | 0.0;
    float rl_rpm = doc["m3508_rpms"]["rl"] | 0.0;
    float rr_rpm = doc["m3508_rpms"]["rr"] | 0.0;

    // Mutexで保護して書き込み
    if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
      target_rpms[0] = static_cast<int16_t>(fl_rpm);
      target_rpms[1] = static_cast<int16_t>(fr_rpm);
      target_rpms[2] = static_cast<int16_t>(rl_rpm);
      target_rpms[3] = static_cast<int16_t>(rr_rpm);
      xSemaphoreGive(DataMutex);
    }
  }

  // pid_gains コマンドの処理
  if (doc.containsKey("pid_gains")) {
    float new_kp = doc["pid_gains"]["kp"] | pid_kp;
    float new_ki = doc["pid_gains"]["ki"] | pid_ki;
    float new_kd = doc["pid_gains"]["kd"] | pid_kd;

    // 範囲チェック
    new_kp = constrain(new_kp, 0.0f, 10.0f);
    new_ki = constrain(new_ki, 0.0f, 1.0f);
    new_kd = constrain(new_kd, 0.0f, 1.0f);

    // Mutexで保護して書き込み
    if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
      pid_kp = new_kp;
      pid_ki = new_ki;
      pid_kd = new_kd;
      xSemaphoreGive(DataMutex);
    }

    Serial.print("PID gains updated: Kp=");
    Serial.print(pid_kp);
    Serial.print(", Ki=");
    Serial.print(pid_ki);
    Serial.print(", Kd=");
    Serial.println(pid_kd);
  }
}

// rspi にフィードバック値を送る
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  // フィードバック値を JSON にシリアライズ（サイズを拡張）
  StaticJsonDocument<768> doc;

  // Mutexで保護して読み込み
  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    // 現在のRPM
    doc["m3508_rpms"]["fl"] = current_rpm_fl;
    doc["m3508_rpms"]["fr"] = current_rpm_fr;
    doc["m3508_rpms"]["rl"] = current_rpm_rl;
    doc["m3508_rpms"]["rr"] = current_rpm_rr;

    // 目標RPM
    doc["target_rpms"]["fl"] = target_rpms[0];
    doc["target_rpms"]["fr"] = target_rpms[1];
    doc["target_rpms"]["rl"] = target_rpms[2];
    doc["target_rpms"]["rr"] = target_rpms[3];

    // 出力電流値
    doc["output_currents"]["fl"] = output_currents_fb[0];
    doc["output_currents"]["fr"] = output_currents_fb[1];
    doc["output_currents"]["rl"] = output_currents_fb[2];
    doc["output_currents"]["rr"] = output_currents_fb[3];

    // P項
    doc["p_terms"]["fl"] = p_terms_fb[0];
    doc["p_terms"]["fr"] = p_terms_fb[1];
    doc["p_terms"]["rl"] = p_terms_fb[2];
    doc["p_terms"]["rr"] = p_terms_fb[3];

    // I項
    doc["i_terms"]["fl"] = i_terms_fb[0];
    doc["i_terms"]["fr"] = i_terms_fb[1];
    doc["i_terms"]["rl"] = i_terms_fb[2];
    doc["i_terms"]["rr"] = i_terms_fb[3];

    // D項
    doc["d_terms"]["fl"] = d_terms_fb[0];
    doc["d_terms"]["fr"] = d_terms_fb[1];
    doc["d_terms"]["rl"] = d_terms_fb[2];
    doc["d_terms"]["rr"] = d_terms_fb[3];

    // 現在のPIDゲイン
    doc["pid_gains"]["kp"] = pid_kp;
    doc["pid_gains"]["ki"] = pid_ki;
    doc["pid_gains"]["kd"] = pid_kd;

    xSemaphoreGive(DataMutex);
  }

  size_t n = serializeJson(doc, send_msg.data.data, send_msg.data.capacity);
  send_msg.data.size = n;

  RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
}

// micro-ROS のセットアップ
void setup_micro_ros() {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL == 1)
  set_microros_serial_transports(Serial);
#elif (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
#endif

  allocator = rcl_get_default_allocator();

  Serial.println("Init support...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("Init node...");
  RCCHECK(rclc_node_init_default(&node, "central_controller", "", &support));

  Serial.println("Init publisher...");
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "robot_feedback"));

  Serial.println("Init subscription...");
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "robot_control"));

  const unsigned int timer_timeout = 20;  // 500ms -> 更新頻度を上げる (50Hz)
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // msg の初期化
  std_msgs__msg__String__init(&recv_msg);
  std_msgs__msg__String__init(&send_msg);

  const size_t kRecvBufferSize = 256;
  const size_t kSendBufferSize = 1024;  // フィードバックデータ増加に対応
  recv_msg.data.data = (char*)malloc(kRecvBufferSize * sizeof(char));
  recv_msg.data.size = 0;
  recv_msg.data.capacity = kRecvBufferSize;

  send_msg.data.data = (char*)malloc(kSendBufferSize * sizeof(char));
  send_msg.data.size = 0;
  send_msg.data.capacity = kSendBufferSize;

  Serial.println("Init executor...");
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  Serial.println("Add timer to executor...");
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("Add subscription to executor...");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg,
                                         &subscription_callback, ON_NEW_DATA));
}

// --------------------------------------------------------------------------------
// Tasks
// --------------------------------------------------------------------------------

// Control Task: High Priority (Core 1)
// CAN受信処理とPD制御ループを担当
void ControlTask(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(3);  // 3ms周期
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // 1. CAN受信処理 (全て取り切る)
    can_comm->process_received_messages();

    // 2. 制御計算
    int32_t output_currents[4] = {0, 0, 0, 0};
    int16_t current_target_rpms[4];

    // 目標値の取得 (Mutex保護)
    if (xSemaphoreTake(DataMutex, 0) == pdTRUE) {  // 待ち時間なし
      current_target_rpms[0] = target_rpms[0];
      current_target_rpms[1] = target_rpms[1];
      current_target_rpms[2] = target_rpms[2];
      current_target_rpms[3] = target_rpms[3];
      xSemaphoreGive(DataMutex);
    } else {
      // ロック取れなければ前回値を使用（あるいは0にするなど安全策）
      // ここでは前回のcurrent_target_rpmsが残っていると仮定せず0初期化
      memset(current_target_rpms, 0, sizeof(current_target_rpms));
    }

    compute_motor_commands(current_target_rpms, output_currents);

    // 【デバッグ用】ターゲットRPMと出力電流を表示 (間引き処理)
    static int debug_count = 0;
    if (debug_count++ > 300) {  // 約1秒ごとに表示 (3ms * 300 ~ 900ms)
      debug_count = 0;
      Serial.print("Target RPM: [");
      Serial.print(current_target_rpms[0]);
      Serial.print(", ");
      Serial.print(current_target_rpms[1]);
      Serial.print(", ");
      Serial.print(current_target_rpms[2]);
      Serial.print(", ");
      Serial.print(current_target_rpms[3]);
      Serial.println("]");

      Serial.print("Out Current: [");
      Serial.print(output_currents[0]);
      Serial.print(", ");
      Serial.print(output_currents[1]);
      Serial.print(", ");
      Serial.print(output_currents[2]);
      Serial.print(", ");
      Serial.print(output_currents[3]);
      Serial.println("]");
    }

    // 3. CAN送信
    uint8_t tx_buf[8];
    milli_amperes_to_bytes(output_currents, tx_buf);
    can_comm->transmit(
        can::CanTxMessage(0x200, {tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                                  tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]}));

    // 周期待機
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Micro-ROS Task: Standard Priority (Core 0)
// Wi-Fi通信とJSONパースを担当
void MicroROSTask(void* pvParameters) {
  // Micro-ROS セットアップ
  setup_micro_ros();

  while (1) {
    // Micro-ROS executor処理
    // タイムアウトを10msにして、タスク切り替えの余地を与える
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("setup start");

  // Mutex作成
  DataMutex = xSemaphoreCreateMutex();

  // CAN 通信の初期化
  can_comm = new can::CanCommunicator();
  can_comm->setup();
  Serial.println("CAN setup complete");

  // CAN 通信のイベントハンドラを登録
  register_can_event_handlers();

  // タスクの作成
  // Core 1 (Application Core) で制御タスクを動かす
  xTaskCreatePinnedToCore(ControlTask, "ControlTask",
                          4096,  // Stack size
                          NULL,
                          configMAX_PRIORITIES - 1,  // 最高優先度
                          &ControlTaskHandle,
                          1  // Core 1
  );

  // Core 0 (Protocol Core) でMicro-ROSを動かす (Wi-Fi処理もCore
  // 0で行われるため)
  xTaskCreatePinnedToCore(
      MicroROSTask, "MicroROSTask",
      8192,  // Stack size (Micro-ROSはスタック食うので大きめに)
      NULL,
      2,  // 標準優先度
      &MicroROSTaskHandle,
      0  // Core 0
  );

  Serial.println("Tasks started");
}

void loop() {
  // メインループは何もしない（タスクで実行）
  vTaskDelay(portMAX_DELAY);
}
