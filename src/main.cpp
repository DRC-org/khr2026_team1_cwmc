#include <Arduino.h>
#include <ArduinoJson.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <stdlib.h>

#include <can/core.hpp>
#include <can/peripheral.hpp>

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

// setup より前に使うので前方宣言しておく
// void subscription_callback(const void* msgin);

volatile int16_t current_rpm_fl = 0;
volatile int16_t current_rpm_fr = 0;
volatile int16_t current_rpm_rl = 0;
volatile int16_t current_rpm_rr = 0;

volatile int16_t target_rpms[4] = {0, 0, 0, 0};

// PD 制御用
#define KP 0.9f
#define KD 100.0f
#define CLAMPING_OUTPUT 5000      // 電流値のクランピング値 [mA]
#define STOP_THRESHOLD_RPM 50.0f  // 停止判定の閾値 [RPM]

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
    delay(5000000);
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
  // TODO: RPM 以外も受け取るようにする？
  can_comm->add_receive_event_listener(
      {0x201, 0x202, 0x203, 0x204},
      [&](const can::CanId id, const std::array<uint8_t, 8> data) {
        int16_t rpm = (data[2] << 8) | data[3];
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
      });
}

// PD 制御
void compute_motor_commands(int16_t target_rpm[4], int32_t out_current[4]) {
  static float prev_rpm_errors[4] = {0, 0, 0, 0};

  float rpm_errors[4];
  rpm_errors[0] = static_cast<float>(target_rpm[0]) - current_rpm_fl;
  rpm_errors[1] = static_cast<float>(target_rpm[1]) - current_rpm_fr;
  rpm_errors[2] = static_cast<float>(target_rpm[2]) - current_rpm_rl;
  rpm_errors[3] = static_cast<float>(target_rpm[3]) - current_rpm_rr;

  for (uint8_t i = 0; i < 4; i++) {
    float error = rpm_errors[i];
    float d_error = error - prev_rpm_errors[i];
    prev_rpm_errors[i] = error;

    int32_t command_current = 0;

    // 目標 RPM が 0 のときのハンチング防止：閾値以下なら出力を 0 にする
    if (target_rpm[i] == 0 /* && fabs(error) < STOP_THRESHOLD_RPM */) {
      command_current = 0;
      prev_rpm_errors[i] = 0.0f;  // 微分項の蓄積もリセット
    } else {
      command_current =
          static_cast<int32_t>(KP * error + KD * d_error);  // PD 制御
    }

    // クランピング
    if (command_current > CLAMPING_OUTPUT) {
      command_current = CLAMPING_OUTPUT;
    } else if (command_current < -CLAMPING_OUTPUT) {
      command_current = -CLAMPING_OUTPUT;
    }
    out_current[i] = command_current;
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

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// rspi からの JSON を受け取る
IRAM_ATTR void subscription_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;

  // msg をプレーンテキストで表示（デバッグ用）
  // Serial.print("Received message: ");
  // Serial.println(msg->data.data);

  // JSON をパース
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, msg->data.data);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  float fl_rpm = doc["m3508_rpms"]["fl"] | 0.0;
  float fr_rpm = doc["m3508_rpms"]["fr"] | 0.0;
  float rl_rpm = doc["m3508_rpms"]["rl"] | 0.0;
  float rr_rpm = doc["m3508_rpms"]["rr"] | 0.0;

  target_rpms[0] = static_cast<int16_t>(fl_rpm);
  target_rpms[1] = static_cast<int16_t>(fr_rpm);
  target_rpms[2] = static_cast<int16_t>(rl_rpm);
  target_rpms[3] = static_cast<int16_t>(rr_rpm);
}

// rspi にフィードバック値を送る
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  // フィードバック値を JSON にシリアライズ
  StaticJsonDocument<256> doc;
  doc["m3508_rpms"]["fl"] = current_rpm_fl;
  doc["m3508_rpms"]["fr"] = current_rpm_fr;
  doc["m3508_rpms"]["rl"] = current_rpm_rl;
  doc["m3508_rpms"]["rr"] = current_rpm_rr;

  size_t n = serializeJson(doc, send_msg.data.data, send_msg.data.capacity);
  send_msg.data.size = n;

  RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
}

// micro-ROS のセットアップ
// @see
// https://github.com/micro-ROS/micro_ros_platformio/blob/main/examples/micro-ros_publisher/src/Main.cpp
// @see https://github.com/micro-ROS/micro-ROS-demos/tree/jazzy/rclc
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

  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout),
                                  timer_callback));

  // msg の初期化（これがないと callback が発火しなくなる）
  std_msgs__msg__String__init(&recv_msg);
  std_msgs__msg__String__init(&send_msg);

  const size_t kBufferSize = 256;
  recv_msg.data.data = (char*)malloc(kBufferSize * sizeof(char));
  recv_msg.data.size = 0;
  recv_msg.data.capacity = kBufferSize;

  send_msg.data.data = (char*)malloc(kBufferSize * sizeof(char));
  send_msg.data.size = 0;
  send_msg.data.capacity = kBufferSize;

  Serial.println("Init executor...");
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  Serial.println("Add timer to executor...");
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  Serial.println("Add subscription to executor...");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg,
                                         &subscription_callback, ON_NEW_DATA));
}

void setup() {
  Serial.begin(115200);

  Serial.println("setup start");

  // CAN 通信の初期化
  can_comm = new can::CanCommunicator();
  can_comm->setup();

  Serial.println("CAN setup complete");

  // micro-ROS のセットアップ
  setup_micro_ros();

  Serial.println("micro-ROS setup complete");

  // CAN 通信のイベントハンドラを登録
  register_can_event_handlers();

  Serial.println("Setup complete");
}

void loop() {
  // micro-ROS の受信メッセージを処理
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  // CAN 通信の受信メッセージを処理（受信キューから 1 つずつ）
  can_comm->process_received_messages();

  // 3ms ごと
  static unsigned long last_control_time = 0;
  if (millis() - last_control_time >= 3) {
    last_control_time = millis();

    // for debug
    // Serial.print("fl: ");
    // Serial.print(current_rpm_fl);
    // Serial.print(", fr: ");
    // Serial.print(current_rpm_fr);
    // Serial.print(", rl: ");
    // Serial.print(current_rpm_rl);
    // Serial.print(", rr: ");
    // Serial.println(current_rpm_rr);

    int32_t output_currents[4] = {0, 0, 0, 0};
    int16_t current_target_rpms[4];

    // volatile な値はローカルにコピー
    noInterrupts();
    current_target_rpms[0] = target_rpms[0];
    current_target_rpms[1] = target_rpms[1];
    current_target_rpms[2] = target_rpms[2];
    current_target_rpms[3] = target_rpms[3];
    interrupts();

    compute_motor_commands(current_target_rpms, output_currents);

    uint8_t tx_buf[8];
    milli_amperes_to_bytes(output_currents, tx_buf);

    can_comm->transmit(
        can::CanTxMessage(0x200, {tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                                  tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]}));
  }
}
