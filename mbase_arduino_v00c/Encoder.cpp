//****************************************************************//
// Simple encoder library
// Created: 26/06/2016 by Henrique Latorre
//****************************************************************//

#include "encoder.h"

Encoder::Encoder():pulse_count(0),timer(millis()),pulse_time(0) {};

Encoder::Encoder(uint8_t pulse_t):pulse_count(0),timer(millis()),pulse_time(pulse_t) { }

Encoder Encoder::operator++ (int) { 
  pulse_count++;
}

Encoder Encoder::operator-- (int) { 
  pulse_count--;
}

void Encoder::reset() {
  pulse_count = 0;
}

void Encoder::update (int8_t delta) { 
  pulse_count += delta;
}
