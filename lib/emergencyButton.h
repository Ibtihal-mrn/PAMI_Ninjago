#pragma once
#include <Arduino.h>

class EmergencyButton {
public:
  explicit EmergencyButton(uint8_t pin);

  void begin();
  bool urgenceActive(); // garde ta logique (HIGH => appuyé)

private:
  uint8_t _pin;
};
