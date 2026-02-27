#include "lsm9ds1_control.hpp"

#include <Wire.h>

// 地磁気偏角 (京田辺別館)
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
#define MAG_DECLINATION 8.16f

namespace lsm9ds1_control {
Lsm9ds1Controller::Lsm9ds1Controller() : yaw_offset_(0.0f) {}

void Lsm9ds1Controller::setup() {
  // Wire.begin() に周波数を渡す（setClock() を後から呼ぶと esp32-hal-i2c-ng
  // でバス状態が壊れる）
  Wire.begin(-1, -1, 400000);
  // Akizuki AE-LSM9DS1 は SA0=0 のため AG=0x6A, M=0x1C（SparkFun
  // ライブラリデフォルトの 0x6B/0x1E とは異なる）
  imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0));
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
  imu.readMag();

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

ImuValues Lsm9ds1Controller::get_all_values() {
  // Available() チェックは no-STOP I2C パターンを使い esp32-hal-i2c-ng
  // と相性が悪いため廃止。 50ms 間隔での呼び出しでは常に新データが存在する。
  imu.readAccel();
  imu.readGyro();
  imu.readMag();

  ImuValues v;

  v.ax = imu.calcAccel(imu.ax);
  v.ay = imu.calcAccel(imu.ay);
  v.az = imu.calcAccel(imu.az);

  v.gx = imu.calcGyro(imu.gx);
  v.gy = imu.calcGyro(imu.gy);
  v.gz = imu.calcGyro(imu.gz);

  // 搭載向き補正（get_raw_yaw と同じ変換）
  float mx = -imu.calcMag(imu.my);
  float my = -imu.calcMag(imu.mx);
  float mz = imu.calcMag(imu.mz);
  v.mx = mx;
  v.my = my;
  v.mz = mz;

  // 加速度計から roll/pitch を算出
  v.roll = atan2f(v.ay, v.az) * RAD_TO_DEG;
  v.pitch = atan2f(-v.ax, sqrtf(v.ay * v.ay + v.az * v.az)) * RAD_TO_DEG;

  // 地磁気から yaw を算出（地磁気偏角補正・ユーザオフセット適用）
  float yaw;
  if (my == 0.0f) {
    yaw = (mx < 0.0f) ? PI : 0.0f;
  } else {
    yaw = atan2f(mx, my);
  }
  yaw -= MAG_DECLINATION * DEG_TO_RAD;
  if (yaw > PI)
    yaw -= 2.0f * PI;
  else if (yaw < -PI)
    yaw += 2.0f * PI;
  yaw *= RAD_TO_DEG;

  float adjusted_yaw = yaw - yaw_offset_;
  if (adjusted_yaw > 180.0f)
    adjusted_yaw -= 360.0f;
  else if (adjusted_yaw < -180.0f)
    adjusted_yaw += 360.0f;
  v.yaw = adjusted_yaw;

  return v;
}
}  // namespace lsm9ds1_control
