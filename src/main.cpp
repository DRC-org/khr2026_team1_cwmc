#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define POWER_LED_PIN 2
#define ERROR_LED_PIN 12
#define LED_1_PIN 14
#define LED_2_PIN 27
#define LED_3_PIN 26
#define LED_4_PIN 25
#define LED_5_PIN 33

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

// Error handle loop
void error_loop() {
  while (1) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(100);
  }
}

void subscription_callback(const void* msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;

  int led_number = msg->data;
  if (led_number < 1 || led_number > 5) {
    digitalWrite(ERROR_LED_PIN, HIGH);
    return;
  }
  int led_pins[5] = {LED_1_PIN, LED_2_PIN, LED_3_PIN, LED_4_PIN, LED_5_PIN};
  digitalWrite(led_pins[led_number - 1], HIGH);
  delay(1000);
  digitalWrite(led_pins[led_number - 1], LOW);
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_4_PIN, OUTPUT);
  pinMode(LED_5_PIN, OUTPUT);

  digitalWrite(POWER_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);
  digitalWrite(LED_1_PIN, HIGH);
  digitalWrite(LED_2_PIN, HIGH);
  digitalWrite(LED_3_PIN, HIGH);
  digitalWrite(LED_4_PIN, HIGH);
  digitalWrite(LED_5_PIN, HIGH);

  delay(2000);

  digitalWrite(POWER_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
  digitalWrite(LED_3_PIN, LOW);
  digitalWrite(LED_4_PIN, LOW);
  digitalWrite(LED_5_PIN, LOW);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  digitalWrite(LED_1_PIN, HIGH);

  // create node
  RCCHECK(rclc_node_init_default(&node, "arduino_sub_test", "", &support));
  digitalWrite(LED_2_PIN, HIGH);

  // create subscription
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "topic"));
  digitalWrite(LED_3_PIN, HIGH);

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  digitalWrite(LED_4_PIN, HIGH);

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         subscription_callback, ON_NEW_DATA));
  digitalWrite(LED_5_PIN, HIGH);
  digitalWrite(POWER_LED_PIN, HIGH);
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
  digitalWrite(LED_3_PIN, LOW);
  digitalWrite(LED_4_PIN, LOW);
  digitalWrite(LED_5_PIN, LOW);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}