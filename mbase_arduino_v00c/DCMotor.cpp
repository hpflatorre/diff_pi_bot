//****************************************************************//
// Simple DC motor drive library
// Created: 26/06/2016 by Henrique Latorre
//****************************************************************//

#include "DCMotor.h"

void DCMotor::set(int16_t speed) {
  if (speed <= -DCMOTOR_PWM_MAX) {
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin, HIGH);
  }
  else if (speed > -DCMOTOR_PWM_MAX && speed <= -DCMOTOR_0_THRESHOLD) {
    analogWrite(pwm_pin, (uint8_t)(DCMOTOR_PWM_MAX + speed));
    digitalWrite(dir_pin, HIGH);
  }
  else if (speed > -DCMOTOR_0_THRESHOLD && speed < DCMOTOR_0_THRESHOLD) {
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin, LOW);
  }
  else if (speed >= DCMOTOR_0_THRESHOLD && speed < DCMOTOR_PWM_MAX) {
    analogWrite(pwm_pin, (uint8_t)speed);
    digitalWrite(dir_pin, LOW);
  }
  else { //if (speed >= DCMOTOR_PWM_MAX) {
    analogWrite(pwm_pin, DCMOTOR_PWM_MAX);
    digitalWrite(dir_pin, LOW);
  }
}

void DCMotor::set_dz(int16_t speed, int16_t dead_zone) {
  if (speed <= -DCMOTOR_PWM_MAX + dead_zone) {
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin, HIGH);
  }
  else if (speed > -DCMOTOR_PWM_MAX + dead_zone && speed <= -DCMOTOR_0_THRESHOLD) {
    analogWrite(pwm_pin, (uint8_t)(DCMOTOR_PWM_MAX + speed - dead_zone));
    digitalWrite(dir_pin, HIGH);
  }
  else if (speed > -DCMOTOR_0_THRESHOLD && speed < DCMOTOR_0_THRESHOLD) {
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin, LOW);
  }
  else if (speed >= DCMOTOR_0_THRESHOLD && speed < DCMOTOR_PWM_MAX - dead_zone) {
    analogWrite(pwm_pin, (uint8_t)speed + dead_zone);
    digitalWrite(dir_pin, LOW);
  }
  else { //if (speed >= DCMOTOR_PWM_MAX) {
    analogWrite(pwm_pin, DCMOTOR_PWM_MAX);
    digitalWrite(dir_pin, LOW);
  }
}
