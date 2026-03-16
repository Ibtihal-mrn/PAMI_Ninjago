#pragma once
#include <Arduino.h>

class Ultrasonic {
public:
  Ultrasonic(uint8_t trig, uint8_t echo, uint8_t trig1, uint8_t echo1, float seuilCm);

  void begin();
  float mesurerDistance();

  float seuil() const { return _seuil; }

private:
  uint8_t _trig, _echo, _trig1, _echo1;
  float _seuil;
};
