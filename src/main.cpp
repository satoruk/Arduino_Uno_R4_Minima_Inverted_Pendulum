#include <Arduino.h>
#include <FspTimer.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <float.h>

ICM_20948_I2C myICM;

// 姿勢推定用のフィルタ
Madgwick filter;

volatile bool updateIMUFlag = false;  // 割り込みフラグ
const float sampleRate = 100.0;       // 100Hz
static FspTimer fsp_timer;

// センサーの値を格納する変数
// 配列で[ax, ay, az, gx, gy, gz, mx, my, mz]の順番
float imuData[9];
float gyroBias[3] = {0.0, 0.0, 0.0};   // X, Y, Z軸のバイアス
float accelBias[3] = {0.0, 0.0, 0.0};  // X, Y, Z軸のバイアス
float magMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
float magMax[3] = {FLT_MIN, FLT_MIN, FLT_MIN};

// calibrateGyro 戻り値は float 型の配列

float *calibrateGyro() {
  const int numSamples = 500;  // キャリブレーションのサンプル数
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("ジャイロスコープのキャリブレーションを開始します...");
  for (int i = 0; i < numSamples; i++) {
    if (!myICM.dataReady()) continue;
    myICM.getAGMT();

    sumX += myICM.gyrX();
    sumY += myICM.gyrY();
    sumZ += myICM.gyrZ();

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

void calibrateAccel() {
  const int numSamples = 500;  // キャリブレーションのサンプル数
  float sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("加速度センサーのキャリブレーションを開始します...");
  for (int i = 0; i < numSamples; i++) {
    if (!myICM.dataReady()) continue;
    myICM.getAGMT();

    sumX += myICM.accX();
    sumY += myICM.accY();
    sumZ += myICM.accZ();

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

void calibrateMag() {
  Serial.println("磁気センサーのキャリブレーションを開始します...");
  Serial.println("デバイスを複数の方向に回転させてください...");

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // 10秒間測定
    if (!myICM.dataReady()) continue;
    myICM.getAGMT();

    float mx = myICM.magX();
    float my = myICM.magY();
    float mz = myICM.magZ();

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

void onTimer(timer_callback_args_t __attribute((unused)) * p_args) {
  updateIMUFlag = true;  // フラグを設定
}

void updateIMU() {
  if (!updateIMUFlag) {
    return;
  }

  updateIMUFlag = false;  // フラグをクリア

  if (!myICM.dataReady()) {
    return;
  }

  myICM.getAGMT();

  // 加速度、ジャイロ、磁気の値を取得
  imuData[0] = myICM.accX();
  imuData[1] = myICM.accY();
  imuData[2] = myICM.accZ();
  imuData[3] = myICM.gyrX();
  imuData[4] = myICM.gyrY();
  imuData[5] = myICM.gyrZ();
  imuData[6] = myICM.magX();
  imuData[7] = myICM.magY();
  imuData[8] = myICM.magZ();

  // Madgwickフィルタを更新
  // filter.updateIMU(imuData[3], imuData[4], imuData[5],  // ジャイロ
  //                  imuData[0], imuData[1], imuData[2]   // 加速度
  // );
  filter.update(imuData[3], imuData[4], imuData[5],   // ジャイロ
                imuData[0], imuData[1], imuData[2],   // 加速度
                imuData[6], imuData[7], imuData[8]);  // 磁気
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  delay(50);

  Serial.println("setup");

  Serial.println("I2Cスキャンを開始します...");
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("デバイスが見つかりました。アドレス: 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("スキャン終了");

  if (myICM.begin(Wire, 0) != ICM_20948_Stat_Ok) {
    Serial.println(
        "ICM-20948の初期化に失敗しました。モジュール接続を確認してください。");
    while (1);
  }
  Serial.println("ICM-20948の初期化に成功しました。");

  delay(100);

  // ジャイロのキャリブレーション
  calibrateGyro();
  myICM.setBiasGyroX((int32_t)(gyroBias[0] * 65536));
  myICM.setBiasGyroY((int32_t)(gyroBias[1] * 65536));
  myICM.setBiasGyroZ((int32_t)(gyroBias[2] * 65536));

  calibrateAccel();
  myICM.setBiasAccelX((int32_t)(accelBias[0] * 65536));
  myICM.setBiasAccelY((int32_t)(accelBias[1] * 65536));
  myICM.setBiasAccelZ((int32_t)(accelBias[2] * 65536));

  calibrateMag();
  myICM.setBiasCPassX((int32_t)((magMin[0] + magMax[0]) / 2 * 65536));
  myICM.setBiasCPassY((int32_t)((magMin[1] + magMax[1]) / 2 * 65536));
  myICM.setBiasCPassZ((int32_t)((magMin[2] + magMax[2]) / 2 * 65536));

  // 50 Hz = 20ms
  // 100 Hz = 10ms
  // 1000 Hz = 1ms
  float freq_hz = 100.0;
  float duty_perc = 25.0;

  filter.begin(freq_hz);  // Hz
  // filter.setGain(1.0);  // これを利用するにはMadgwickFilterのソース修正が必要
  // [参考] https://blog2.studiok-i.net/2489.html

  uint8_t timer_type;
  int8_t timer_ch = FspTimer::get_available_timer(timer_type);
  if (timer_ch < 0) return;
  fsp_timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_ch, freq_hz, duty_perc,
                  onTimer, nullptr);
  fsp_timer.setup_overflow_irq();
  fsp_timer.open();
  fsp_timer.start();
  delay(100);
}

void loop() {
  updateIMU();

  // 100ms ごとに姿勢を表示
  static unsigned long prevTime = 0;
  if (millis() - prevTime < 100) {
    return;
  }
  prevTime = millis();

  // データを表示
  // 0: V 固定
  // 1-3: 加速度XYZ
  // 4-6: ジャイロXYZ
  // 7-9: 磁気XYZ
  // 10-12: ロール,ピッチ,ヨー
  Serial.print("V,");

  // ループを使ってimuDataの値を表示
  for (int i = 0; i < 9; i++) {
    Serial.print(imuData[i], 2);
    Serial.print(",");
  }

  // 姿勢角を計算
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // 結果を表示
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(yaw, 2);
  Serial.println();
}
