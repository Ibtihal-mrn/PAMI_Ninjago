#pragma once
#include <Arduino.h>

class Encoders {
public:
  Encoders(uint8_t pinG, uint8_t pinD);

  void begin();
  void reset();

  long ticksG() const { return _ticksG; }
  long ticksD() const { return _ticksD; }

  // ISR
  void isrG();
  void isrD();

  // helpers statiques pour attachInterrupt
  static void attach(Encoders* instance);
  static void ISR_G();
  static void ISR_D();

private:
  uint8_t _pinG, _pinD;
  volatile long _ticksG = 0;
  volatile long _ticksD = 0;

  static Encoders* _self;
};
