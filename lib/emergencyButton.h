#ifndef EMERGENCYBUTTON_H
#define EMERGENCYBUTTON_H

#include <Arduino.h>

// Adapter selon le cablage:
// - HIGH: urgence active quand la broche lit HIGH
// - LOW : urgence active quand la broche lit LOW
// Cas fréquent bouton d'arrêt d'urgence en contact NF vers GND + INPUT_PULLUP:
// - au repos: contact ferme -> pin LOW
// - appui: contact ouvre -> pin HIGH
#define EMERGENCY_ACTIVE_LEVEL HIGH

void emergencyButton_init();
bool emergencyButton_isPressed();  // exactement ce nom
uint8_t emergencyButton_rawRead();

#endif