#include "MotorDriver.h"

#include <Arduino.h>

MotorDriver::MotorDriver() {}
MotorDriver::~MotorDriver() {
  // 生成したら delete する（2回目以降の delete を防ぐチェックも）
  if (forwardPWM) {
    forwardPWM->pulse_perc(0.0f);
    delete forwardPWM;
  }
  if (reversePWM) {
    reversePWM->pulse_perc(0.0f);
    delete reversePWM;
  }
}

void MotorDriver::begin(int forwardPin, int reversePin, float pwmFrequency,
                        float minDuty) {
  this->pwmFrequency = pwmFrequency;
  this->minDuty = minDuty;

  forwardPWM = new PwmOut(forwardPin);
  reversePWM = new PwmOut(reversePin);

  forwardPWM->begin(pwmFrequency, 0.0f);
  reversePWM->begin(pwmFrequency, 0.0f);

  state = MotorState::STOP;
  stateTime = micros();
}

MotorState MotorDriver::forward(int duty) {
  if (state == MotorState::FORWARD && this->duty == duty) {
    return state;
  }

  state = MotorState::FORWARD;
  this->duty = duty;
  forwardPWM->pulse_perc(this->duty);
  reversePWM->pulse_perc(0.0f);
  return state;

  // unsigned long now = micros();
  // // 1sec = 1,000 ms
  // // 1sec = 1,000,000 μs
  // int initTime = 500;
  // this->duty = duty;

  // if (state == MotorState::FORWARD_INIT) {
  //   if (now - stateTime < initTime) {
  //     // 始動時間が経過していない場合は何もしない
  //     return state;
  //   }

  //   // 始動時間が経過したら指定のデューティー比で回転させる
  //   state = MotorState::FORWARD;
  //   forwardPWM->pulse_perc(this->duty);
  //   return state;
  // }

  // if (state == MotorState::FORWARD) {
  //   // すでに正転している場合はデューティー比を変更する
  //   forwardPWM->pulse_perc(this->duty);
  //   return state;
  // }

  // // モーターを始動するのに一度勢いをつける
  // state = MotorState::FORWARD_INIT;
  // stateTime = micros();
  // forwardPWM->pulse_perc(100.0f);
  // reversePWM->pulse_perc(0.0f);
  // return state;
}

MotorState MotorDriver::reverse(int duty) {
  if (state == MotorState::REVERSE && this->duty == duty) {
    return state;
  }

  state = MotorState::REVERSE;
  this->duty = duty;
  forwardPWM->pulse_perc(0.0f);
  reversePWM->pulse_perc(this->duty);
  return state;

  // unsigned long now = micros();
  // // 1sec = 1,000 ms
  // // 1sec = 1,000,000 μs
  // int initTime = 1000;
  // this->duty = duty;

  // if (state == MotorState::REVERSE_INIT) {
  //   if (now - stateTime < initTime) {
  //     // 始動時間が経過していない場合は何もしない
  //     return state;
  //   }

  //   // 始動時間が経過したら指定のデューティー比で回転させる
  //   state = MotorState::REVERSE;
  //   reversePWM->pulse_perc(this->duty);
  //   return state;
  // }

  // if (state == MotorState::REVERSE) {
  //   // すでに正転している場合はデューティー比を変更する
  //   reversePWM->pulse_perc(this->duty);
  //   return state;
  // }

  // // モーターを始動するのに一度勢いをつける
  // state = MotorState::REVERSE_INIT;
  // stateTime = micros();
  // forwardPWM->pulse_perc(0.0f);
  // reversePWM->pulse_perc(100.0f);
  // return state;
}

MotorState MotorDriver::stop() {
  if (state == MotorState::STOP) {
    return state;
  }

  forwardPWM->pulse_perc(0.0f);
  reversePWM->pulse_perc(0.0f);
  state = MotorState::STOP;
  // stateTime = micros();
  return state;
}

MotorState MotorDriver::update() {
  if (state == MotorState::FORWARD) {
    return forward(this->duty);
  }

  if (state == MotorState::REVERSE) {
    return reverse(this->duty);
  }
  // if (state == MotorState::FORWARD_INIT) {
  //   return forward(this->duty);
  // }

  // if (state == MotorState::REVERSE_INIT) {
  //   return reverse(this->duty);
  // }

  return state;
}