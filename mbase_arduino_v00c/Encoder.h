//****************************************************************//
// Simple encoder library
// Created: 26/06/2016 by Henrique Latorre
//****************************************************************//

#ifndef ENCODER_H
#define ENCODER_H
#include <arduino.h>

class Encoder {
  private:
    int32_t pulse_count;
    volatile uint32_t timer;
    uint8_t pulse_time;
  public:
    Encoder();
    Encoder(uint8_t pulse_t);
    int32_t read();
    void reset();
    void update(int8_t);
    void pulse (bool forward);
    Encoder operator++ (int);
    Encoder operator-- (int);
};

inline int32_t Encoder::read() { 
  return pulse_count;
}

inline void Encoder::pulse (bool forward) {
  if(millis() - timer > pulse_time){
    if (forward){
      pulse_count++;
    }
    else {
      pulse_count--;
    }    
  }
  timer = millis();
}

#endif
