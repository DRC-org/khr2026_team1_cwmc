#pragma once

#include <SparkFunLSM9DS1.h>

namespace lsm9ds1_control {

struct ImuValues {
  float ax, ay, az;        // 加速度 [g]
  float gx, gy, gz;        // 角速度 [deg/s]
  float mx, my, mz;        // 磁場 [Gauss]（搭載向き補正済み）
  float roll, pitch, yaw;  // 姿勢角 [deg]
};

class Lsm9ds1Controller {
 public:
  Lsm9ds1Controller();
  void setup();

  ImuValues get_all_values();
  float get_yaw();
  void reset_yaw();

 private:
  LSM9DS1 imu;
  float yaw_offset_;

  float get_raw_yaw();
};
}  // namespace lsm9ds1_control