#include "../lib/emergencyButton.h"

EmergencyButton::EmergencyButton(uint8_t pin) : _pin(pin) {}

void EmergencyButton::begin() {
  pinMode(_pin, INPUT_PULLUP);
}

bool EmergencyButton::urgenceActive() {
  return digitalRead(_pin) == HIGH; // == ton code
}
