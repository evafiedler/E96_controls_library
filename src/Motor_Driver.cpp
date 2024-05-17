#include "Motor_Driver.h"

Motor_Driver::Motor_Driver(uint8_t in1, uint8_t in2, uint8_t ena)
{
    pin1 = in1;
    pin2 = in2;
    pwm = ena;
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    run(0);
}

void Motor_Driver::run(float speed)
{
    if (speed < 0)
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwm, (uint8_t)-speed * 255);
    }
    else if (speed > 0)
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwm, (uint8_t)speed * 255);
    }
    else
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
}

void Motor_Driver::brake()
{
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, HIGH);
}