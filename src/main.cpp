#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <builtin_interfaces/msg/time.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <can/core.hpp>
#include <can/peripheral.hpp>
#include <lsm9ds1_control.hpp>

// タスクハンドル定義
TaskHandle_t MicroROSTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;
SemaphoreHandle_t DataMutex = NULL;

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;
geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

can::CanCommunicator* can_comm;
lsm9ds1_control::Lsm9ds1Controller imu_controller;

volatile int16_t current_rpm_fl = 0;
volatile int16_t current_rpm_fr = 0;
volatile int16_t current_rpm_rl = 0;
volatile int16_t current_rpm_rr = 0;

volatile int16_t target_rpms[4] = {0, 0, 0, 0};

// メカナムロボットパラメータ
const float WHEEL_RADIUS = 0.04925f; // [m]
const float LX = 0.1725f;            // [m] ロボット中心からホイールまでの前後距離 (Machine center to wheel center X)
const float LY = 0.2425f;            // [m] ロボット中心からホイールまでの左右距離 (Machine center to wheel center Y)
const float GEAR_RATIO = 19.2032f;   // ギア比 (M3508 P19)

// オドメトリ用変数
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;
unsigned long last_odom_time = 0;

// PID 制御用
#define KP 1.0f   // 比例ゲイン（要調整）
#define KI 0.05f  // 積分ゲイン（要調整）
#define KD 50.0f  // 微分ゲイン（要調整）

#define INTEGRAL_LIMIT 1000.0f  // 積分項の上限（ワインドアップ防止）
#define CLAMPING_OUTPUT 5000      // 電流値のクランピング値 [mA] (M3508: Max 16384 -> 20A)
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
    delay(5000); // 5秒待機に変更
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
        if(xSemaphoreTake(DataMutex, 0) == pdTRUE) {
            switch (id) {
            case 0x201: current_rpm_fl = rpm; break;
            case 0x202: current_rpm_fr = rpm; break;
            case 0x203: current_rpm_rl = rpm; break;
            case 0x204: current_rpm_rr = rpm; break;
            default: break;
            }
            xSemaphoreGive(DataMutex);
        }
      });
}

// PID 制御（積分項を追加）
void compute_motor_commands(int16_t target_rpm[4], int32_t out_current[4]) {
  static float prev_rpm_errors[4] = {0, 0, 0, 0};
  static float integral_errors[4] = {0, 0, 0, 0};  // 追加：積分項

  // Mutexで保護して読み込み
  int16_t current_rpms[4];
  if(xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
      current_rpms[0] = current_rpm_fl;
      current_rpms[1] = current_rpm_fr;
      current_rpms[2] = current_rpm_rl;
      current_rpms[3] = current_rpm_rr;
      xSemaphoreGive(DataMutex);
  } else {
      return; 
  }

  float rpm_errors[4];
  rpm_errors[0] = static_cast<float>(target_rpm[0]) - current_rpms[0];
  rpm_errors[1] = static_cast<float>(target_rpm[1]) - current_rpms[1];
  rpm_errors[2] = static_cast<float>(target_rpm[2]) - current_rpms[2];
  rpm_errors[3] = static_cast<float>(target_rpm[3]) - current_rpms[3];

  for (uint8_t i = 0; i < 4; i++) {
    float error = rpm_errors[i];
    
    // 積分項の更新（ワインドアップ防止付き）
    if (target_rpm[i] != 0) {
      integral_errors[i] += error;
      // 積分項の制限
      if (integral_errors[i] > INTEGRAL_LIMIT) {
        integral_errors[i] = INTEGRAL_LIMIT;
      } else if (integral_errors[i] < -INTEGRAL_LIMIT) {
        integral_errors[i] = -INTEGRAL_LIMIT;
      }
    } else {
      integral_errors[i] = 0.0f;  // 目標0のときはリセット
    }
    
    // 微分項
    float d_error = error - prev_rpm_errors[i];
    prev_rpm_errors[i] = error;

    int32_t command_current = 0;

    // 目標 RPM が 0 のときのハンチング防止
    if (target_rpm[i] == 0) {
      command_current = 0;
      prev_rpm_errors[i] = 0.0f;
      integral_errors[i] = 0.0f;  // 積分項もリセット
    } else {
      // PID 制御
      command_current = static_cast<int32_t>(
          KP * error + 
          KI * integral_errors[i] + 
          KD * d_error
      );
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

// cmd_vel (m/s, rad/s) → モーターRPM に変換 (メカナム逆運動学)
void cmd_vel_to_rpm(float vx, float vy, float vth, int16_t out_rpms[4]) {
    // 逆運動学計算
    // ホイール配置:
    // FL(0)   FR(1)
    //    Robot
    // RL(2)   RR(3)
    
    // 各ホイールの目標速度 [m/s]
    float v_fl = vx - vy - (LX + LY) * vth;
    float v_fr = vx + vy + (LX + LY) * vth;
    float v_rl = vx + vy - (LX + LY) * vth;
    float v_rr = vx - vy + (LX + LY) * vth;
    
    // m/s -> RPM 変換 (ギア比込み)
    // RPM = (v * 60 * GEAR_RATIO) / (2 * PI * R)
    float rad_to_rpm_coeff = (60.0f * GEAR_RATIO) / (2.0f * PI * WHEEL_RADIUS);
    
    out_rpms[0] = static_cast<int16_t>(v_fl * rad_to_rpm_coeff); // FL
    out_rpms[1] = static_cast<int16_t>(v_fr * rad_to_rpm_coeff); // FR
    out_rpms[2] = static_cast<int16_t>(v_rl * rad_to_rpm_coeff); // RL
    out_rpms[3] = static_cast<int16_t>(v_rr * rad_to_rpm_coeff); // RR
}

// エンコーダー値からオドメトリを計算
void update_odometry() {
    unsigned long current_time = millis();
    float dt = (current_time - last_odom_time) / 1000.0f; // [s]
    last_odom_time = current_time;
    
    if (dt > 0.1f) dt = 0.02f; // 初回や異常値対策
    
    // 現在のRPMを取得（Mutex保護）
    int16_t current_rpms[4];
    if(xSemaphoreTake(DataMutex, 0) == pdTRUE) {
        current_rpms[0] = current_rpm_fl;
        current_rpms[1] = current_rpm_fr;
        current_rpms[2] = current_rpm_rl;
        current_rpms[3] = current_rpm_rr;
        xSemaphoreGive(DataMutex);
    } else {
        return;
    }
    
    // RPM から ホイール速度 [m/s] への変換係数
    // v = (RPM / 60 / GEAR_RATIO) * 2 * PI * R
    float rpm_to_vel_coeff = (2.0f * PI * WHEEL_RADIUS) / (60.0f * GEAR_RATIO);
    
    float v_fl = current_rpms[0] * rpm_to_vel_coeff;
    float v_fr = current_rpms[1] * rpm_to_vel_coeff;
    float v_rl = current_rpms[2] * rpm_to_vel_coeff;
    float v_rr = current_rpms[3] * rpm_to_vel_coeff;
    
    // メカナム順運動学: ロボットローカル座標系の速度 (vx, vy)
    float vx  = (v_fl + v_fr + v_rl + v_rr) / 4.0f;
    float vy  = (-v_fl + v_fr + v_rl - v_rr) / 4.0f;
    // float vth_wheels = (-v_fl + v_fr - v_rl + v_rr) / (4.0f * (LX + LY));
    
    // IMUからYaw角を取得してオドメトリの角度に設定
    float yaw_deg = imu_controller.get_yaw();
    float new_odom_theta = yaw_deg * DEG_TO_RAD;

    // 角速度を算出（前回の角度との差分）
    static float prev_odom_theta = 0.0f;
    float delta_theta = new_odom_theta - prev_odom_theta;
    
    // 角度の正規化処理（-PI ~ PIの境界またぎ対策）
    if (delta_theta > PI) delta_theta -= 2.0f * PI;
    if (delta_theta < -PI) delta_theta += 2.0f * PI;

    float vth_imu = delta_theta / dt;
    prev_odom_theta = new_odom_theta;
    odom_theta = new_odom_theta;

    // 現在の角度(odom_theta)を使って、ローカル速度(vx, vy)をグローバル移動量(delta_x, delta_y)に変換
    // 回転行列:
    // [ dx ] = [ cos -sin ] [ vx ]
    // [ dy ] = [ sin  cos ] [ vy ]
    float cos_th = cos(odom_theta);
    float sin_th = sin(odom_theta);
    
    float delta_x = (vx * cos_th - vy * sin_th) * dt;
    float delta_y = (vx * sin_th + vy * cos_th) * dt;
    
    odom_x += delta_x;
    odom_y += delta_y;
    
    // オドメトリメッセージに格納
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    
    // クォータニオンなど (変更なし)
    odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0f);
    
    // 速度 (Twist) - ローカル座標系の速度
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vth_imu;
    odom_msg.pose.pose.position.z = 0.0;
    
    // クォータニオン（Z軸回転のみ）
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    // z, w は上で設定済み
    
    // 速度 (Twist) - 残りの成分を0に
    // linear.x, y, angular.z は上で設定済み
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    
    // タイムスタンプ（簡易版）
    odom_msg.header.stamp.sec = current_time / 1000;
    odom_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
}

// cmd_vel コールバック
IRAM_ATTR void cmd_vel_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float vth = msg->angular.z;
    
    // 速度をRPMに変換
    int16_t new_target_rpms[4];
    cmd_vel_to_rpm(vx, vy, vth, new_target_rpms);
    
    // Mutexで保護して書き込み
    if(xSemaphoreTake(DataMutex, portMAX_DELAY) == pdTRUE) {
        target_rpms[0] = new_target_rpms[0];
        target_rpms[1] = new_target_rpms[1];
        target_rpms[2] = new_target_rpms[2];
        target_rpms[3] = new_target_rpms[3];
        xSemaphoreGive(DataMutex);
    }
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
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));

  // 変更: Subscriber (cmd_vel)
  Serial.println("Init cmd_vel subscription...");
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel"));

  // 変更: Publisher (odom)
  Serial.println("Init odom publisher...");
  RCCHECK(rclc_publisher_init_default(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "/odom"));

  // メッセージ初期化
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  nav_msgs__msg__Odometry__init(&odom_msg);
  
  // オドメトリのframe_id設定
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = strlen("odom") + 1;
  
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.child_frame_id.capacity = strlen("base_link") + 1;

  // Executor初期化
  Serial.println("Init executor...");
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // タイマー削除で1に変更

  // Subscriptionのみ追加
  Serial.println("Add cmd_vel subscription to executor...");
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
                                         &cmd_vel_callback, ON_NEW_DATA));
}

// --------------------------------------------------------------------------------
// Tasks
// --------------------------------------------------------------------------------

// Control Task: High Priority (Core 1)
// CAN受信処理とPID制御ループを担当
void ControlTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(3); // 3ms周期
    xLastWakeTime = xTaskGetTickCount();
    
    // オドメトリ送信用カウンタ
    int odom_counter = 0;

    while(1) {
        // 1. CAN受信処理 (全て取り切る)
        can_comm->process_received_messages();

        // 2. 制御計算
        int32_t output_currents[4] = {0, 0, 0, 0};
        int16_t current_target_rpms[4];

        // 目標値の取得 (Mutex保護)
        if(xSemaphoreTake(DataMutex, 0) == pdTRUE) { // 待ち時間なし
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
        if (debug_count++ > 300) { // 約1秒ごとに表示 (3ms * 300 ~ 900ms)
            debug_count = 0;
            Serial.print("Target RPM: [");
            Serial.print(current_target_rpms[0]); Serial.print(", ");
            Serial.print(current_target_rpms[1]); Serial.print(", ");
            Serial.print(current_target_rpms[2]); Serial.print(", ");
            Serial.print(current_target_rpms[3]); Serial.println("]");
            
            Serial.print("Out Current: [");
            Serial.print(output_currents[0]); Serial.print(", ");
            Serial.print(output_currents[1]); Serial.print(", ");
            Serial.print(output_currents[2]); Serial.print(", ");
            Serial.print(output_currents[3]); Serial.println("]");
        }

        // 3. CAN送信
        uint8_t tx_buf[8];
        milli_amperes_to_bytes(output_currents, tx_buf);
        can_comm->transmit(
            can::CanTxMessage(0x200, {tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                                      tx_buf[4], tx_buf[5], tx_buf[6], tx_buf[7]}));
        
        // 4. オドメトリ更新＆送信（50Hz = 20ms周期）
        odom_counter++;
        if (odom_counter >= 7) { // 3ms * 7 ≈ 21ms
            odom_counter = 0;
            update_odometry();
            rcl_publish(&odom_publisher, &odom_msg, NULL);
        }
        
        // 周期待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Micro-ROS Task: Standard Priority (Core 0)
// Wi-Fi通信とcmd_vel受信を担当
void MicroROSTask(void *pvParameters) {
    // Micro-ROS セットアップ
    setup_micro_ros();

    while(1) {
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

  // IMU初期化
  imu_controller.setup();
  imu_controller.reset_yaw();
  Serial.println("IMU setup complete");

  // CAN 通信のイベントハンドラを登録
  register_can_event_handlers();

  // タスクの作成
  // Core 1 (Application Core) で制御タスクを動かす
  xTaskCreatePinnedToCore(
      ControlTask,
      "ControlTask",
      4096,  // Stack size
      NULL,
      configMAX_PRIORITIES - 1, // 最高優先度
      &ControlTaskHandle,
      1 // Core 1
  );

  // Core 0 (Protocol Core) でMicro-ROSを動かす (Wi-Fi処理もCore 0で行われるため)
  xTaskCreatePinnedToCore(
      MicroROSTask,
      "MicroROSTask",
      8192,  // Stack size (Micro-ROSはスタック食うので大きめに)
      NULL,
      2,     // 標準優先度
      &MicroROSTaskHandle,
      0 // Core 0
  );

  Serial.println("Tasks started");
}

void loop() {
  // メインループは何もしない（タスクで実行）
  vTaskDelay(portMAX_DELAY);
}
