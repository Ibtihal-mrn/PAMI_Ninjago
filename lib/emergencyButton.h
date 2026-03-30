#ifndef EMERGENCYBUTTON_H
#define EMERGENCYBUTTON_H

#include <Arduino.h>

// Adapter selon le cablage:
// - HIGH: urgence active quand la broche lit HIGH
// - LOW : urgence active quand la broche lit LOW
#define EMERGENCY_ACTIVE_LEVEL HIGH

void emergencyButton_init();
bool emergencyButton_isPressed();  // exactement ce nom

#endif