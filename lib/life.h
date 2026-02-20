#pragma once
#include <Arduino.h>
#include <Servo.h>

class Life {
public:
  Life(unsigned long dureeMs, Servo& servo, int posInit, int posFin);

  void begin();
  bool check(); // retourne true si fin de vie déclenchée

private:
  unsigned long _duree;
  Servo& _servo;
  int _posInit, _posFin;

  unsigned long _startTime = 0;
  unsigned long _lastPrint = 0;
  bool _declenchee = false;
};
