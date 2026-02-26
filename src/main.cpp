#include <Arduino.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <robot_msgs/msg/detail/feedback_pid_terms__functions.h>
#include <robot_msgs/msg/detail/imu_values__functions.h>
#include <robot_msgs/msg/detail/pid_gains__functions.h>
#include <robot_msgs/msg/wheel_message.h>
#include <stdlib.h>

#include <can/core.hpp>
#include <can/peripheral.hpp>
#include <lsm9ds1_control/core.cpp>

TaskHandle_t MicroROSTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;
SemaphoreHandle_t DataMutex = NULL;

// 開発時に WiFi 接続する用
#if (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
char ssid[] = "DRC";
char psk[] = "kumachan";
IPAddress agent_ip(192, 168, 1, 101);
size_t agent_port = 8888;
#endif

#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      (void)temp_rc;               \
    }                              \
  }

can::CanCommunicator* can_comm;
lsm9ds1_control::Lsm9ds1Controller lsm_controller;

// micro-ROS オブジェクト
rcl_publisher_t pub_feedback;
rcl_subscription_t sub_control;
rcl_timer_t timer_feedback;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// メッセージバッファ
robot_msgs__msg__WheelMessage feedback_msg;
robot_msgs__msg__WheelMessage control_msg;

// 各ホイールの状態
struct WheelRPMs {
  float_t fl;
  float_t fr;
  float_t rl;
  float_t rr;
};

volatile WheelRPMs target = {0, 0, 0, 0};
volatile WheelRPMs current = {0, 0, 0, 0};

// 各モータの電流値
struct MotorCurrents {
  int32_t fl;
  int32_t fr;
  int32_t rl;
  int32_t rr;
};

volatile MotorCurrents output_currents = {0, 0, 0, 0};

// PID ゲイン
struct PIDGains {
  float kp;
  float ki;
  float kd;
};

volatile PIDGains pid_gains = {0.5f, 0.05f, 0.0f};

// PID Term
struct PIDTerms {
  float p;
  float i;
  float d;
};

struct FeedbackPIDTerms {
  PIDTerms fl;
  PIDTerms fr;
  PIDTerms rl;
  PIDTerms rr;
};

volatile FeedbackPIDTerms pid_terms;

// volatile int16_t current_rpm_fl = 0;
// volatile int16_t current_rpm_fr = 0;
// volatile int16_t current_rpm_rl = 0;
// volatile int16_t current_rpm_rr = 0;

// volatile int16_t target_rpms[4] = {0, 0, 0, 0};

// volatile int32_t output_currents_fb[4] = {0, 0, 0, 0};
// volatile float p_terms_fb[4] = {0, 0, 0, 0};
// volatile float i_terms_fb[4] = {0, 0, 0, 0};
// volatile float d_terms_fb[4] = {0, 0, 0, 0};

// ランタイムで変更可能な PID ゲイン
// volatile float pid_kp = 0.5f;
// volatile float pid_ki = 0.05f;
// volatile float pid_kd = 0.0f;
#define DT 0.003f  // 制御周期 [秒]

#define CLAMPING_OUTPUT 5000     // 電流クランプ [mA] (M3508: Max 16384 -> 20A)
#define INTEGRAL_LIMIT 10000.0f  // Anti-windup 積分上限

enum class AgentState { WAITING, CONNECTED, DISCONNECTED };

void register_can_event_handlers() {
  // M3508 のフィードバック RPM を受け取る
  can_comm->add_receive_event_listener(
      {0x201, 0x202, 0x203, 0x204},
      [&](const can::CanId id, const std::array<uint8_t, 8> data) {
        int16_t rpm = (data[2] << 8) | data[3];

        // volatile int16_t への書き込みは ESP32 でアトミック。
        // Mutex を使うと Core 0 の micro-ROS タスクとの競合で
        // フィードバックがドロップされるため、ここでは使わない。
        switch (id) {
          case 0x201:
            current.fl = rpm;
            break;
          case 0x202:
            current.fr = rpm;
            break;
          case 0x203:
            current.rl = rpm;
            break;
          case 0x204:
            current.rr = rpm;
            break;
        }
      });
}

void compute_motor_commands(WheelRPMs target_rpm, int32_t out_current[4]) {
  static float prev_rpm_errors[4] = {0, 0, 0, 0};
  static float integral_errors[4] = {0, 0, 0, 0};

  WheelRPMs local_rpms;
  float local_kp, local_ki, local_kd;
  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    local_rpms.fl = current.fl;
    local_rpms.fr = current.fr;
    local_rpms.rl = current.rl;
    local_rpms.rr = current.rr;
    local_kp = pid_gains.kp;
    local_ki = pid_gains.ki;
    local_kd = pid_gains.kd;
    xSemaphoreGive(DataMutex);
  }

  const float target_arr[4] = {target_rpm.fl, target_rpm.fr, target_rpm.rl,
                               target_rpm.rr};
  const float current_arr[4] = {local_rpms.fl, local_rpms.fr, local_rpms.rl,
                                local_rpms.rr};
  float p_terms_arr[4], i_terms_arr[4], d_terms_arr[4];

  for (uint8_t i = 0; i < 4; i++) {
    float error = target_arr[i] - current_arr[i];
    float d_error = (error - prev_rpm_errors[i]) / DT;
    prev_rpm_errors[i] = error;

    // target=0 のとき積分をリセットするが、出力は 0 に固定しない。
    // P/D 項によるアクティブブレーキを継続することで即時停止させる。
    if (target_arr[i] == 0.0f) {
      integral_errors[i] = 0.0f;
    }

    float p_term = local_kp * error;
    float i_term = local_ki * integral_errors[i];
    float d_term = local_kd * d_error;
    float pid_output = p_term + i_term + d_term;

    int32_t command_current = static_cast<int32_t>(pid_output);
    if (command_current > CLAMPING_OUTPUT) {
      command_current = CLAMPING_OUTPUT;
    } else if (command_current < -CLAMPING_OUTPUT) {
      command_current = -CLAMPING_OUTPUT;
    }

    // 方向性 Anti-windup:
    // 飽和方向と同じ向きに error がある場合のみ積分を止める。
    // 逆方向 (脱飽和方向) の error なら積分を続け、素早く飽和を解消させる。
    if (target_arr[i] != 0.0f) {
      bool winding_up_pos = (pid_output > CLAMPING_OUTPUT) && (error > 0.0f);
      bool winding_up_neg = (pid_output < -CLAMPING_OUTPUT) && (error < 0.0f);

      if (!winding_up_pos && !winding_up_neg) {
        integral_errors[i] += error * DT;
        if (integral_errors[i] > INTEGRAL_LIMIT) {
          integral_errors[i] = INTEGRAL_LIMIT;
        } else if (integral_errors[i] < -INTEGRAL_LIMIT) {
          integral_errors[i] = -INTEGRAL_LIMIT;
        }
      }
    }

    out_current[i] = command_current;
    p_terms_arr[i] = p_term;
    i_terms_arr[i] = i_term;
    d_terms_arr[i] = d_term;
  }

  if (xSemaphoreTake(DataMutex, 0) == pdTRUE) {
    output_currents.fl = out_current[0];
    output_currents.fr = out_current[1];
    output_currents.rl = out_current[2];
    output_currents.rr = out_current[3];
    pid_terms.fl.p = p_terms_arr[0];
    pid_terms.fl.i = i_terms_arr[0];
    pid_terms.fl.d = d_terms_arr[0];
    pid_terms.fr.p = p_terms_arr[1];
    pid_terms.fr.i = i_terms_arr[1];
    pid_terms.fr.d = d_terms_arr[1];
    pid_terms.rl.p = p_terms_arr[2];
    pid_terms.rl.i = i_terms_arr[2];
    pid_terms.rl.d = d_terms_arr[2];
    pid_terms.rr.p = p_terms_arr[3];
    pid_terms.rr.i = i_terms_arr[3];
    pid_terms.rr.d = d_terms_arr[3];
    xSemaphoreGive(DataMutex);
  }
}

// C620 ESC 用: mA 値を CAN データフレームに変換
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

// wheel_control topic の callback
IRAM_ATTR void on_control_command(const void* msg_in) {
  const auto* cmd = static_cast<const robot_msgs__msg__WheelMessage*>(msg_in);

  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    target.fl = cmd->m3508_rpms.fl;
    target.fr = cmd->m3508_rpms.fr;
    target.rl = cmd->m3508_rpms.rl;
    target.rr = cmd->m3508_rpms.rr;

    if (cmd->m3508_gains.size > 0) {
      pid_gains.kp = cmd->m3508_gains.data[0].kp;
      pid_gains.ki = cmd->m3508_gains.data[0].ki;
      pid_gains.kd = cmd->m3508_gains.data[0].kd;
    }

    xSemaphoreGive(DataMutex);
  }
}

// 50 ms ごとにフィードバックを送信する
IRAM_ATTR void timer_feedback_callback(rcl_timer_t* timer,
                                       int64_t /*last_call_time*/) {
  if (timer == nullptr) return;

  if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
    feedback_msg.m3508_rpms.fl = current.fl;
    feedback_msg.m3508_rpms.fr = current.fr;
    feedback_msg.m3508_rpms.rl = current.rl;
    feedback_msg.m3508_rpms.rr = current.rr;

    feedback_msg.m3508_gains.data[0].kp = pid_gains.kp;
    feedback_msg.m3508_gains.data[0].ki = pid_gains.ki;
    feedback_msg.m3508_gains.data[0].kd = pid_gains.kd;

    feedback_msg.m3508_terms.data[0].fl.p = pid_terms.fl.p;
    feedback_msg.m3508_terms.data[0].fl.i = pid_terms.fl.i;
    feedback_msg.m3508_terms.data[0].fl.d = pid_terms.fl.d;
    feedback_msg.m3508_terms.data[0].fr.p = pid_terms.fr.p;
    feedback_msg.m3508_terms.data[0].fr.i = pid_terms.fr.i;
    feedback_msg.m3508_terms.data[0].fr.d = pid_terms.fr.d;
    feedback_msg.m3508_terms.data[0].rl.p = pid_terms.rl.p;
    feedback_msg.m3508_terms.data[0].rl.i = pid_terms.rl.i;
    feedback_msg.m3508_terms.data[0].rl.d = pid_terms.rl.d;
    feedback_msg.m3508_terms.data[0].rr.p = pid_terms.rr.p;
    feedback_msg.m3508_terms.data[0].rr.i = pid_terms.rr.i;
    feedback_msg.m3508_terms.data[0].rr.d = pid_terms.rr.d;

    xSemaphoreGive(DataMutex);
  }

  lsm9ds1_control::ImuValues imu = lsm_controller.get_all_values();
  feedback_msg.lsm9ds1_values.data[0].ax = imu.ax;
  feedback_msg.lsm9ds1_values.data[0].ay = imu.ay;
  feedback_msg.lsm9ds1_values.data[0].az = imu.az;
  feedback_msg.lsm9ds1_values.data[0].gx = imu.gx;
  feedback_msg.lsm9ds1_values.data[0].gy = imu.gy;
  feedback_msg.lsm9ds1_values.data[0].gz = imu.gz;
  feedback_msg.lsm9ds1_values.data[0].mx = imu.mx;
  feedback_msg.lsm9ds1_values.data[0].my = imu.my;
  feedback_msg.lsm9ds1_values.data[0].mz = imu.mz;
  feedback_msg.lsm9ds1_values.data[0].roll = imu.roll;
  feedback_msg.lsm9ds1_values.data[0].pitch = imu.pitch;
  feedback_msg.lsm9ds1_values.data[0].yaw = imu.yaw;
  feedback_msg.lsm9ds1_values.size = 1;

  feedback_msg.m3508_gains.size = 1;
  feedback_msg.m3508_terms.size = 1;

  RCSOFTCHECK(rcl_publish(&pub_feedback, &feedback_msg, NULL));
}

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK)
    return false;
  if (rclc_node_init_default(&node, "cwmc_node", "", &support) != RCL_RET_OK)
    return false;

  if (rclc_publisher_init_default(
          &pub_feedback, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, WheelMessage),
          "wheel_feedback") != RCL_RET_OK)
    return false;

  if (rclc_subscription_init_default(
          &sub_control, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(robot_msgs, msg, WheelMessage),
          "wheel_control") != RCL_RET_OK)
    return false;

  if (rclc_timer_init_default(&timer_feedback, &support, RCL_MS_TO_NS(50),
                              timer_feedback_callback) != RCL_RET_OK)
    return false;

  robot_msgs__msg__WheelMessage__init(&control_msg);
  robot_msgs__msg__WheelMessage__init(&feedback_msg);

  // __init は Sequence の data を NULL・容量 0 で初期化するため、
  // data[0] アクセス前に容量 1 で明示的にアロケートする
  robot_msgs__msg__IMUValues__Sequence__init(&feedback_msg.lsm9ds1_values, 1);
  robot_msgs__msg__PIDGains__Sequence__init(&control_msg.m3508_gains, 1);
  robot_msgs__msg__PIDGains__Sequence__init(&feedback_msg.m3508_gains, 1);
  robot_msgs__msg__FeedbackPIDTerms__Sequence__init(&control_msg.m3508_terms,
                                                    1);
  robot_msgs__msg__FeedbackPIDTerms__Sequence__init(&feedback_msg.m3508_terms,
                                                    1);

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) !=
      RCL_RET_OK)
    return false;
  if (rclc_executor_add_subscription(&executor, &sub_control, &control_msg,
                                     &on_control_command,
                                     ON_NEW_DATA) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_timer(&executor, &timer_feedback) != RCL_RET_OK)
    return false;

  return true;
}

static void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor);
  rcl_timer_fini(&timer_feedback);
  rcl_subscription_fini(&sub_control, &node);
  rcl_publisher_fini(&pub_feedback, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  robot_msgs__msg__WheelMessage__fini(&control_msg);
  robot_msgs__msg__WheelMessage__fini(&feedback_msg);
}

// Control Task: Core 1, 最高優先度
void ControlTask(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(3);
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // 制御計算
    int32_t output_currents[4] = {0, 0, 0, 0};
    WheelRPMs local_current_rpms;
    WheelRPMs local_target_rpms;

    if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
      local_current_rpms.fl = current.fl;
      local_current_rpms.fr = current.fr;
      local_current_rpms.rl = current.rl;
      local_current_rpms.rr = current.rr;

      local_target_rpms.fl = target.fl;
      local_target_rpms.fr = target.fr;
      local_target_rpms.rl = target.rl;
      local_target_rpms.rr = target.rr;

      xSemaphoreGive(DataMutex);
    }

    compute_motor_commands(local_target_rpms, output_currents);

// Serial transport 使用時は Serial がバイナリプロトコルに占有されるため、
// デバッグ出力を有効にするとフレームが壊れて切断の原因になる
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
    static int debug_count = 0;
    if (debug_count++ > 300) {
      debug_count = 0;
      Serial.print("Target RPM: [");
      Serial.print(local_target_rpms.fl);
      Serial.print(", ");
      Serial.print(local_target_rpms.fr);
      Serial.print(", ");
      Serial.print(local_target_rpms.rl);
      Serial.print(", ");
      Serial.print(local_target_rpms.rr);
      Serial.println("]");

      Serial.print("Current RPM: [");
      Serial.print(local_current_rpms.fl);
      Serial.print(", ");
      Serial.print(local_current_rpms.fr);
      Serial.print(", ");
      Serial.print(local_current_rpms.rl);
      Serial.print(", ");
      Serial.print(local_current_rpms.rr);
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
#endif

    // CAN 送信
    uint8_t tx_buf[8];
    milli_amperes_to_bytes(output_currents, tx_buf);
    can_comm->transmit(
        can::CanTxMessage(0x200, {tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                                  tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]}));

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Micro-ROS Task: Core 0, 標準優先度
void MicroROSTask(void* pvParameters) {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL == 1)
  set_microros_serial_transports(Serial);
#elif (MICRO_ROS_TRANSPORT_ARDUINO_WIFI == 1)
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  // パワーセーブを無効化: DTIM スリープ中のパケット遅延（最大 100ms）が
  // micro-ROS keepalive の UDP タイムアウトを引き起こすのを防ぐ。
  esp_wifi_set_ps(WIFI_PS_NONE);
#endif

  AgentState agent_state = AgentState::WAITING;
  uint32_t ping_counter = 0;

  while (1) {
    switch (agent_state) {
      case AgentState::WAITING:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          if (create_entities()) {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
            Serial.println("micro-ROS: connected");
#endif
            ping_counter = 0;
            agent_state = AgentState::CONNECTED;
          } else {
            destroy_entities();
          }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        break;

      case AgentState::CONNECTED:
        // 約 1 秒ごとに agent の死活確認
        if (++ping_counter >= 100) {
          ping_counter = 0;
          if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
            Serial.println("micro-ROS: agent disconnected");
#endif
            agent_state = AgentState::DISCONNECTED;
            break;
          }
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
        break;

      case AgentState::DISCONNECTED:
        // 切断時にモーターを安全停止
        if (xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
          target.fl = 0;
          target.fr = 0;
          target.rl = 0;
          target.rr = 0;
          xSemaphoreGive(DataMutex);
        }
        destroy_entities();
        agent_state = AgentState::WAITING;
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
  Serial.println("setup start");
#endif

  DataMutex = xSemaphoreCreateMutex();

  lsm_controller.setup();

  // CAN 通信の初期化（フィルタなし = 全受信）
  can_comm = new can::CanCommunicator();
  can_comm->setup();
#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
  Serial.println("CAN setup complete");
#endif

  register_can_event_handlers();

  // Core 1 (Application Core) で制御タスクを実行
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 4096, NULL,
                          configMAX_PRIORITIES - 1, &ControlTaskHandle, 1);

  // Core 0 (Protocol Core) で Micro-ROS を実行（Wi-Fi も Core 0
  // で処理されるため）
  xTaskCreatePinnedToCore(MicroROSTask, "MicroROSTask", 8192, NULL, 2,
                          &MicroROSTaskHandle, 0);

#if (MICRO_ROS_TRANSPORT_ARDUINO_SERIAL != 1)
  Serial.println("Tasks started");
#endif
}

void loop() { vTaskDelay(portMAX_DELAY); }
