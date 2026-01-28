#pragma once

#include <SparkFunLSM9DS1.h>

namespace lsm9ds1_control {
class Lsm9ds1Controller {
 public:
  Lsm9ds1Controller();
  void setup();

  float get_yaw();
  void reset_yaw();

 private:
  LSM9DS1 imu;
  float yaw_offset_;

  float get_raw_yaw();
};
}  // namespace lsm9ds1_control