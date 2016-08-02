//****************************************************************//
// Simple DC motor drive library
// Created: 26/06/2016 by Henrique Latorre
//****************************************************************//
#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <arduino.h>

#define DCMOTOR_PWM_MAX       255
#define DCMOTOR_0_THRESHOLD   5

class DCMotor {
  private:
    uint8_t pwm_pin;
    uint8_t dir_pin;
  public:
    DCMotor(uint8_t pwm_pin, uint8_t dir_pin);
    void set(int16_t speed);
    void set_dz(int16_t speed, int16_t dead_zone);
};

inline DCMotor::DCMotor(uint8_t pwm, uint8_t dir):pwm_pin(pwm),dir_pin(dir) {
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, LOW);
  analogWrite(pwm_pin, 0);
}

#endif
