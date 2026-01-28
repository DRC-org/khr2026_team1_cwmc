#include "lsm9ds1_control.hpp"

#include <Wire.h>

// 利用値の地磁気補正
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
#define MAG_DECLINATION 8.16f  // 京田辺別館

namespace lsm9ds1_control {
Lsm9ds1Controller::Lsm9ds1Controller() : yaw_offset_(0.0f) {}

void Lsm9ds1Controller::setup() {
  Wire.begin();
  Wire.setClock(400000);
  imu.begin();
}

float Lsm9ds1Controller::get_yaw() {
  float raw_yaw = get_raw_yaw();
  float adjusted_yaw = raw_yaw - yaw_offset_;

  // -180〜180度に補正
  if (adjusted_yaw > 180.0f) {
    adjusted_yaw -= 360.0f;
  } else if (adjusted_yaw < -180.0f) {
    adjusted_yaw += 360.0f;
  }

  return adjusted_yaw;
}

void Lsm9ds1Controller::reset_yaw() { yaw_offset_ = get_raw_yaw(); }

float Lsm9ds1Controller::get_raw_yaw() {
  // TODO: ↓ 要るか要らないか確認
  // if (imu.gyroAvailable()) {
  //   imu.readGyro();
  // }
  // if (imu.accelAvailable()) {
  //   imu.readAccel();
  // }
  if (imu.magAvailable()) {
    imu.readMag();
  }

  float mx = -imu.my;
  float my = -imu.mx;
  float mz = imu.mz;

  float yaw;
  if (my == 0) {
    yaw = (mx < 0) ? PI : 0;
  } else {
    yaw = atan2f(mx, my);
  }

  yaw -= MAG_DECLINATION * DEG_TO_RAD;

  if (yaw > PI) {
    yaw -= 2 * PI;
  } else if (yaw < -PI) {
    yaw += 2 * PI;
  }

  yaw *= RAD_TO_DEG;
  return yaw;
}
}  // namespace lsm9ds1_control