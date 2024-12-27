#ifndef IMU_H  // インクルードガード
#define IMU_H

#include <Arduino.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <float.h>

class IMU {
 public:
  float imuData[9];
  float gyroBias[3] = {0.0, 0.0, 0.0};   // X, Y, Z軸のバイアス
  float accelBias[3] = {0.0, 0.0, 0.0};  // X, Y, Z軸のバイアス
  float magMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float magMax[3] = {FLT_MIN, FLT_MIN, FLT_MIN};

  Madgwick filter;

  IMU();

  int begin(TwoWire& wirePort = Wire, bool ad0val = true,
            uint8_t ad0pin = ICM_20948_ARD_UNUSED_PIN);
  void update();

  void calibrate();
  float* calibrateGyro();
  void calibrateAccel();
  void calibrateMag();

 private:
  ICM_20948_I2C _icm;
};

#endif