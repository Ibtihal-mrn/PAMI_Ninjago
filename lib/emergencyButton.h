#ifndef EMERGENCYBUTTON_H
#define EMERGENCYBUTTON_H

#include <Arduino.h>

// Adapter selon le cablage:
// - HIGH: urgence active quand la broche lit HIGH
// - LOW : urgence active quand la broche lit LOW
// Avec INPUT_PULLUP (voir emergencyButton.cpp), le cas le plus courant est:
// bouton appuyé => broche à GND => lecture LOW.
#define EMERGENCY_ACTIVE_LEVEL LOW

void emergencyButton_init();
bool emergencyButton_isPressed();  // exactement ce nom

#endif