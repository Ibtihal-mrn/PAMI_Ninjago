#pragma once
#include <Arduino.h>

void safety_init(int obstacle_cm, uint16_t sonar_period_ms);
void safety_update();

bool safety_isTriggered();
void safety_clearIfSafe();

// Détails (pour reproduire le comportement "pause obstacle" du sketch)
bool safety_isEmergencyActive();
bool safety_isObstacleActive();
int safety_getObstacleThresholdCm();
void safety_getLastDistancesCm(int &out_us1, int &out_us2);