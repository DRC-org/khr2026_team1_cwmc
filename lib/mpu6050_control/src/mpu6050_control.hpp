#pragma once

#include <Adafruit_MPU6050.h>

namespace mpu6050_control {

struct ImuValues {
  float ax, ay, az;        // 加速度 [g]
  float gx, gy, gz;        // 角速度 [deg/s]
  float mx, my, mz;        // MPU6050 には磁気センサなし（常に 0）
  float roll, pitch, yaw;  // 姿勢角 [deg]（yaw は磁気センサなしのため常に 0）
};

class Mpu6050Controller {
 public:
  void setup();
  ImuValues get_all_values();

 private:
  Adafruit_MPU6050 mpu_;
};
}  // namespace mpu6050_control
