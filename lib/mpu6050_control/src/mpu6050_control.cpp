#include "mpu6050_control.hpp"

#include <Wire.h>

namespace mpu6050_control {

void Mpu6050Controller::setup() {
  Wire.begin(-1, -1, 400000);
  mpu_.begin(0x68, &Wire);

  mpu_.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

ImuValues Mpu6050Controller::get_all_values() {
  sensors_event_t accel, gyro, temp;
  mpu_.getEvent(&accel, &gyro, &temp);

  ImuValues v;

  // Adafruit ライブラリは m/s² で返すため g に変換
  constexpr float MS2_TO_G = 1.0f / 9.80665f;
  v.ax = accel.acceleration.x * MS2_TO_G;
  v.ay = accel.acceleration.y * MS2_TO_G;
  v.az = accel.acceleration.z * MS2_TO_G;

  // Adafruit ライブラリは rad/s で返すため deg/s に変換
  v.gx = gyro.gyro.x * RAD_TO_DEG;
  v.gy = gyro.gyro.y * RAD_TO_DEG;
  v.gz = gyro.gyro.z * RAD_TO_DEG;

  // MPU6050 には磁気センサがない
  v.mx = 0.0f;
  v.my = 0.0f;
  v.mz = 0.0f;

  v.roll = atan2f(v.ay, v.az) * RAD_TO_DEG;
  v.pitch = atan2f(-v.ax, sqrtf(v.ay * v.ay + v.az * v.az)) * RAD_TO_DEG;
  v.yaw = 0.0f;

  return v;
}
}  // namespace mpu6050_control
