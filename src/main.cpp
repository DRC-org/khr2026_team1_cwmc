#include <Arduino.h>
#include <ArduinoJson.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <can/core.hpp>
#include <can/peripheral.hpp>

rcl_subscription_t subscriber;
std_msgs__msg__String msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

can::CanCommunicator* can_comm;

// setup より前に使うので前方宣言しておく
void subscription_callback(const void* msgin);

float current_rpm_fl = 0.0;
float current_rpm_fr = 0.0;
float current_rpm_rl = 0.0;
float current_rpm_rr = 0.0;

// P 制御用
#define KP 1.0
#define CLAMPING_OUTPUT 5000  // 電流値のクランピング値 [mA]

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
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
IPAddress agent_ip(192, 168, 0, 100);
size_t agent_port = 8888;
#endif

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

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(
      rclc_node_init_default(&node, "m3508_motor_controller", "", &support));

  // rspi からのサブスクリプションを初期化
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "robot_control"));

  // エグゼキュータを初期化
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // サブスクリプションをエグゼキュータに追加
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         subscription_callback, ON_NEW_DATA));
}

void register_can_event_handlers() {
  // M3508 のフィードバック値を受け取る
  // TODO: RPM 以外も受け取るようにする？
  can_comm->add_receive_event_listener(
      {0x201, 0x202, 0x203, 0x204},
      [&](const can::CanId id, const std::array<uint8_t, 8> data) {
        int16_t rpm = (data[2] << 8) | data[3];
        switch (id) {
          case 0x201:
            current_rpm_fl = static_cast<float>(rpm);
            break;
          case 0x202:
            current_rpm_fr = static_cast<float>(rpm);
            break;
          case 0x203:
            current_rpm_rl = static_cast<float>(rpm);
            break;
          case 0x204:
            current_rpm_rr = static_cast<float>(rpm);
            break;
          default:
            break;
        }
      });
}

// P 制御
void compute_motor_commands(int16_t target_rpm[4], int32_t out_current[4]) {
  float rpm_errors[4];
  rpm_errors[0] = static_cast<float>(target_rpm[0]) - current_rpm_fl;
  rpm_errors[1] = static_cast<float>(target_rpm[1]) - current_rpm_fr;
  rpm_errors[2] = static_cast<float>(target_rpm[2]) - current_rpm_rl;
  rpm_errors[3] = static_cast<float>(target_rpm[3]) - current_rpm_rr;

  for (uint8_t i = 0; i < 4; i++) {
    int32_t command_current =
        static_cast<int32_t>(KP * rpm_errors[i]);  // P 制御
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

// rspi からの JSON を受け取る
void subscription_callback(const void* msgin) {
  Serial.println("Received message from rspi");
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;

  // JSON をパース
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, msg->data.data);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  int16_t fl_rpm = doc["m3508_rpms"]["fl"] | 0;
  int16_t fr_rpm = doc["m3508_rpms"]["fr"] | 0;
  int16_t rl_rpm = doc["m3508_rpms"]["rl"] | 0;
  int16_t rr_rpm = doc["m3508_rpms"]["rr"] | 0;

  int16_t target_rpms[4] = {fl_rpm, fr_rpm, rl_rpm, rr_rpm};
  int32_t output_currents[4] = {0, 0, 0, 0};

  compute_motor_commands(target_rpms, output_currents);

  uint8_t tx_buf[8];
  milli_amperes_to_bytes(output_currents, tx_buf);

  // TODO: ↓ この冗長な書き方を何とかしたい
  can_comm->transmit(
      can::CanTxMessage(0x200, {tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                                tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]}));

  Serial.print("fl: ");
  Serial.print(fl_rpm);
  Serial.print(" fr: ");
  Serial.print(fr_rpm);
  Serial.print(" rl: ");
  Serial.print(rl_rpm);
  Serial.print(" rr: ");
  Serial.println(rr_rpm);
}

void setup() {
  Serial.begin(115200);

  // CAN 通信の初期化
  can_comm = new can::CanCommunicator();
  can_comm->setup();

  // micro-ROS のセットアップ
  setup_micro_ros();

  // CAN 通信のイベントハンドラを登録
  register_can_event_handlers();
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
