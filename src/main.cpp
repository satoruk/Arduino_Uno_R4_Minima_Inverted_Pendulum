#include <Arduino.h>
#include <FspTimer.h>

#include "IMU.h"
#include "MotorDriver.h"

#ifndef DEBUG  // インクルードガード
#define DEBUG 0
#endif

MotorDriver motorDriver;

// モーターピン定義
const int forwardPin = 10;        // IN1
const int reversePin = 9;         // IN2
const float pwmFrequency = 1000;  // PWM 周波数 1kHz

float roll = 0;
float Kp = 1900;       // Pゲイン
float Ki = 1000;       // Iゲイン
float Kd = 5;          // Dゲイン
float target = 56.0f;  // 目標値。モジュールが横置きなら0前後、縦置きなら90前後

unsigned long preTime;
float dt;
float P, I, D, U, preP;
float power = 0;     // モータの出力(PID計算結果)
float duty = 60.0f;  // pwmデューティ比

float minDuty = 41.0f;
int stoptheta = 40;  // 倒れすぎたらモータを止める角度

IMU imu;
volatile bool updateIMUFlag = false;  // 割り込みフラグ
static FspTimer fsp_timer;

// log("test", 123.45f) でログが出る関数
void log(float now, const char* str, float val) {
  Serial.print(">");
  Serial.print(str);
  Serial.print(":");
  Serial.print(now);
  Serial.print(":");
  Serial.println(val);
}
void log(float now, const char* str, const char* val) {
  Serial.print(">");
  Serial.print(str);
  Serial.print(":");
  Serial.print(now);
  Serial.print(":");
  Serial.print(val);
  Serial.println("|t");
}

// シリアルからのゲインを動的に設定する関数
void processSerialData() {
  char inputBuffer[20];  // シリアルからの入力を格納するバッファ
  int currentIndex = 0;  // バッファの現在の位置
  float values[4];       // 分割された値を格納する配列

  // "1,2,3"のような形式で入力されたデータを分割して配列に格納
  for (int i = 0; i < 4; i++) {
    currentIndex = 0;
    while (Serial.available()) {
      char receivedChar = Serial.read();
      // Serial.println(receivedChar);
      if (receivedChar == ',' ||
          receivedChar == '\0') {          // カンマを区切り文字として処理
        inputBuffer[currentIndex] = '\0';  // 文字列を終端する
        break;
      } else {
        inputBuffer[currentIndex] = receivedChar;  // バッファに文字を追加
      }
      currentIndex++;
    }
    values[i] = atof(inputBuffer);  // 文字列を整数に変換して配列に格納
    /*if (currentIndex >= pidArraySize) {
      //
    配列がすでにいっぱいの場合、何か処理を行うか、エラーメッセージを送信できます
      currentIndex = 0;  // バッファをリセット
    }*/
  }
  while (Serial.available()) {
    Serial.read();
  }
  Kp = values[0];
  Ki = values[1];
  Kd = values[2];
  target = values[3];

  Serial.print("Kp:");
  Serial.print(Kp);
  Serial.print(", Ki:");
  Serial.print(Ki);
  Serial.print(", Kd:");
  Serial.print(Kd);
  Serial.print(", target:");
  Serial.println(target);
}

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

  motorDriver.begin(forwardPin, reversePin, pwmFrequency, minDuty);
  delay(100);
  motorDriver.stop();

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

int count = 0;

void loop() {
  imu.update();
  // if (updateIMUFlag) {
  //   updateIMUFlag = false;
  // }

  if (Serial.available() > 0) {
    processSerialData();  // シリアルに入力があったら処理を呼ぶ
  }

  // 100ms ごとに姿勢を表示
  unsigned long now = millis();
  static unsigned long prevTime = 0;
  bool flagDisplay = false;
  if (now - prevTime >= 300) {
    prevTime = now;
    flagDisplay = true;
  }

#if DEBUG
  if (flagDisplay) {
    // 姿勢角を計算
    float roll = imu.filter.getRoll();
    float pitch = imu.filter.getPitch();
    float yaw = imu.filter.getYaw();

    // 結果を表示
    log(now, "roll", roll);
    log(now, "pitch", pitch);
    log(now, "yaw", yaw);
  }
#endif

  roll = imu.filter.getRoll();  // 角度取得

  unsigned long unow = micros();
  dt = (unow - preTime) * 0.000001;  // 処理時間を求める

  preTime = unow;  // 処理時間を記録

  // PID制御
  // 目標角度から現在の角度を引いて偏差を求める
  P = (target - roll) / 90;  // -90~90を取るので180で割って-1.0~1.0にする

  I += P * dt;          // 偏差を積分する
  D = (P - preP) / dt;  // 偏差を微分する
  preP = P;             // 偏差を記録する

  // 積分部分が大きくなりすぎると出力が飽和するので大きくなり過ぎたら0に戻す(アンチワインドアップ)
  if (150 < abs(I * Ki)) I = 0;

#if DEBUG
  if (flagDisplay) {
    log(now, "KP*P", Kp * P);
    log(now, "KI*I", Ki * I);
    log(now, "KD*D", Kd * D);
    log(now, "P", P);
    log(now, "I", I);
    log(now, "D", D);
    log(now, "target", target);
    log(now, "limit p", -stoptheta + target);
    log(now, "limit n", stoptheta + target);
    log(now, "dt", dt);
  }
#endif

  // 角度を検知してモータを動作させる。(倒立振子の主動作)
  // 出力を計算する
  power = (int)(Kp * P + Ki * I + Kd * D);

  duty = map(constrain((int)abs(power), 0, 350), 0, 350, 40, 100);
  // duty = (int)(constrain((int)abs(power), V_MIN, V_MAX));  //
  // 255に制限　飽和する

#if DEBUG
  if (flagDisplay) {
    log(now, "power", power);
    log(now, "duty", duty);
  }
#endif
  // roll = 49
  // stoptheta =
  // target = 49

  if (roll < -stoptheta + target || stoptheta + target < roll) {
    // 倒れすぎたら停止
    motorDriver.stop();
    P = 0;
    I = 0;
    D = 0;
#if DEBUG
    if (flagDisplay) {
      log(now, "rotate", "standby");
    }
#endif
  } else {
    // if (duty < minDuty) {
    //   // あまりにも小さいと回らないので停止
    //   motorDriver.stop();
    //   log(now, "rotate", "stop");
    // } else if (power < 0) {
    if (power < 0) {
      motorDriver.reverse(duty);
#if DEBUG
      if (flagDisplay) {
        log(now, "rotate", "reverse");
      }
#endif
    } else if (0 < power) {
      motorDriver.forward(duty);
#if DEBUG
      if (flagDisplay) {
        log(now, "rotate", "forward");
      }
#endif
    }
  }
}
