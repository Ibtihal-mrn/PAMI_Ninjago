#include <Arduino.h>
#include "../lib/emergencyButton.h"

#define EBTN_PIN 8

void emergencyButton_init()
{
    pinMode(EBTN_PIN, INPUT_PULLUP);
}

bool emergencyButton_isPressed()
{
    return digitalRead(EBTN_PIN) == EMERGENCY_ACTIVE_LEVEL;
}