#include <Arduino.h>
#include <FspTimer.h>

#include "IMU.h"

IMU imu;
volatile bool updateIMUFlag = false;  // 割り込みフラグ
static FspTimer fsp_timer;

void onTimer(timer_callback_args_t __attribute((unused)) * p_args) {
  updateIMUFlag = true;  // フラグを設定
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  delay(100);

  if (imu.begin(Wire, 0, 0) != 0) {
    Serial.println(
        "ICM-20948の初期化に失敗しました。モジュール接続を確認してください。");
    while (1);
  }
  Serial.println("ICM-20948の初期化に成功しました。");

  delay(100);
  imu.calibrate();

  // 50 Hz = 20ms
  // 100 Hz = 10ms
  // 1000 Hz = 1ms
  float freq_hz = 100.0;
  float duty_perc = 25.0;

  uint8_t timer_type;
  int8_t timer_ch = FspTimer::get_available_timer(timer_type);
  if (timer_ch < 0) return;
  fsp_timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_ch, freq_hz, duty_perc,
                  onTimer, nullptr);
  fsp_timer.setup_overflow_irq();
  fsp_timer.open();
  fsp_timer.start();
  delay(100);
  Serial.println("setup");
}

void loop() {
  if (updateIMUFlag) {
    imu.update();
    updateIMUFlag = false;
  }

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
    Serial.print(imu.imuData[i], 2);
    Serial.print(",");
  }

  // 姿勢角を計算
  float roll = imu.filter.getRoll();
  float pitch = imu.filter.getPitch();
  float yaw = imu.filter.getYaw();

  // 結果を表示
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(yaw, 2);
  Serial.println();
}
