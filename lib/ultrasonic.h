#pragma once
#include <Arduino.h>

class Ultrasonic {
public:
  Ultrasonic(uint8_t trig, uint8_t echo, float seuilCm);

  void begin();
  float mesurerDistance();

  float seuil() const { return _seuil; }

private:
  uint8_t _trig, _echo;
  float _seuil;
};
