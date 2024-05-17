#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "Arduino.h"

class Motor_Driver
{
public:
  Motor_Driver(uint8_t in1, uint8_t in2, uint8_t ena);
  void run(float speed);
  void brake();

private:
  uint8_t pin1;
  uint8_t pin2;
  uint8_t pwm;
};

#endif