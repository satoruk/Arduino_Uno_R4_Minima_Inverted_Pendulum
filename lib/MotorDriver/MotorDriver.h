#ifndef MotorDriver_H  // インクルードガード
#define MotorDriver_H

#include <Arduino.h>

#include "pwm.h"

enum class MotorState : uint8_t {
  STOP,
  FORWARD,
  FORWARD_INIT,
  REVERSE,
  REVERSE_INIT
};

class MotorDriver {
 public:
  float pwmFrequency;
  float minDuty;
  float duty;
  MotorState state;
  unsigned long stateTime;

  MotorDriver();
  ~MotorDriver();

  void begin(int forwardPin, int reversePin, float pwmFrequency, float minDuty);
  MotorState forward(int duty);
  MotorState reverse(int duty);
  MotorState stop();
  MotorState update();

 private:
  PwmOut* forwardPWM;
  PwmOut* reversePWM;
};

#endif