#include "IMU.h"

IMU::IMU() {}

int IMU::begin(TwoWire& wirePort, bool ad0val, uint8_t ad0pin) {
  if (_icm.begin(wirePort, ad0val, ad0pin) != ICM_20948_Stat_Ok) {
    Serial.println(
        "ICM-20948の初期化に失敗しました。モジュール接続を確認してください。");
    return -1;
  }

  filter.begin(100);
  return 0;
}

void IMU::update() {
  if (!_icm.dataReady()) {
    return;
  }

  _icm.getAGMT();

  // 加速度、ジャイロ、磁気の値を取得
  imuData[0] = _icm.accX();
  imuData[1] = _icm.accY();
  imuData[2] = _icm.accZ();
  imuData[3] = _icm.gyrX();
  imuData[4] = _icm.gyrY();
  imuData[5] = _icm.gyrZ();
  imuData[6] = _icm.magX();
  imuData[7] = _icm.magY();
  imuData[8] = _icm.magZ();

  // Madgwickフィルタを更新
  // filter.updateIMU(imuData[3], imuData[4], imuData[5],  // ジャイロ
  //                  imuData[0], imuData[1], imuData[2]   // 加速度
  // );
  filter.update(imuData[3], imuData[4], imuData[5],   // ジャイロ
                imuData[0], imuData[1], imuData[2],   // 加速度
                imuData[6], imuData[7], imuData[8]);  // 磁気
}

void IMU::calibrate() {
  calibrateGyro();
  _icm.setBiasGyroX((int32_t)(gyroBias[0] * 65536));
  _icm.setBiasGyroY((int32_t)(gyroBias[1] * 65536));
  _icm.setBiasGyroZ((int32_t)(gyroBias[2] * 65536));

  calibrateAccel();
  _icm.setBiasAccelX((int32_t)(accelBias[0] * 65536));
  _icm.setBiasAccelY((int32_t)(accelBias[1] * 65536));
  _icm.setBiasAccelZ((int32_t)(accelBias[2] * 65536));

  calibrateMag();
  _icm.setBiasCPassX((int32_t)((magMin[0] + magMax[0]) / 2 * 65536));
  _icm.setBiasCPassY((int32_t)((magMin[1] + magMax[1]) / 2 * 65536));
  _icm.setBiasCPassZ((int32_t)((magMin[2] + magMax[2]) / 2 * 65536));

  // 50 Hz = 20ms
  // 100 Hz = 10ms
  // 1000 Hz = 1ms
  float freq_hz = 100.0;

  filter.begin(freq_hz);  // Hz
}

float* IMU::calibrateGyro() {
  const int numSamples = 500;  // キャリブレーションのサンプル数
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("ジャイロスコープのキャリブレーションを開始します...");
  for (int i = 0; i < numSamples; i++) {
    if (!_icm.dataReady()) continue;
    _icm.getAGMT();

    sumX += _icm.gyrX();
    sumY += _icm.gyrY();
    sumZ += _icm.gyrZ();

    delay(5);  // サンプリング間隔
  }

  gyroBias[0] = sumX / numSamples;
  gyroBias[1] = sumY / numSamples;
  gyroBias[2] = sumZ / numSamples;

  Serial.println("キャリブレーション完了:");
  Serial.print("Gyro Bias X: ");
  Serial.println(gyroBias[0]);
  Serial.print("Gyro Bias Y: ");
  Serial.println(gyroBias[1]);
  Serial.print("Gyro Bias Z: ");
  Serial.println(gyroBias[2]);
  return gyroBias;
}
void IMU::calibrateAccel() {
  const int numSamples = 500;  // キャリブレーションのサンプル数
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("加速度センサーのキャリブレーションを開始します...");
  for (int i = 0; i < numSamples; i++) {
    if (!_icm.dataReady()) continue;
    _icm.getAGMT();

    sumX += _icm.accX();
    sumY += _icm.accY();
    sumZ += _icm.accZ();

    delay(5);  // サンプリング間隔
  }

  accelBias[0] = sumX / numSamples;
  accelBias[1] = sumY / numSamples;
  accelBias[2] = (sumZ / numSamples) - 9.81;  // 重力加速度を引く

  Serial.println("キャリブレーション完了:");
  Serial.print("Accel Bias X: ");
  Serial.println(accelBias[0]);
  Serial.print("Accel Bias Y: ");
  Serial.println(accelBias[1]);
  Serial.print("Accel Bias Z: ");
  Serial.println(accelBias[2]);
}

void IMU::calibrateMag() {
  Serial.println("磁気センサーのキャリブレーションを開始します...");
  Serial.println("デバイスを複数の方向に回転させてください...");

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // 10秒間測定
    if (!_icm.dataReady()) continue;
    _icm.getAGMT();

    float mx = _icm.magX();
    float my = _icm.magY();
    float mz = _icm.magZ();

    magMin[0] = min(magMin[0], mx);
    magMin[1] = min(magMin[1], my);
    magMin[2] = min(magMin[2], mz);

    magMax[0] = max(magMax[0], mx);
    magMax[1] = max(magMax[1], my);
    magMax[2] = max(magMax[2], mz);

    delay(50);
  }

  Serial.println("キャリブレーション完了:");
  Serial.print("Mag Min X: ");
  Serial.println(magMin[0]);
  Serial.print("Mag Min Y: ");
  Serial.println(magMin[1]);
  Serial.print("Mag Min Z: ");
  Serial.println(magMin[2]);
  Serial.print("Mag Max X: ");
  Serial.println(magMax[0]);
  Serial.print("Mag Max Y: ");
  Serial.println(magMax[1]);
  Serial.print("Mag Max Z: ");
  Serial.println(magMax[2]);
}