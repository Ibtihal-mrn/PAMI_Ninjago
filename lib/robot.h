#pragma once
#include <Arduino.h>
#include "motors.h"
#include "encoders.h"
#include "safety.h"

class Robot {
public:
  Robot(Motors& motors, Encoders& enc, Safety& safety);

  void avancer_ticks(long ticks_cible);
  void tourner_gauche_ticks(long ticks_cible);
  void avancer_cm(float distance_cm);
  void tourner_gauche_deg(float deg);

private:
  Motors& _motors;
  Encoders& _enc;
  Safety& _safety;

  // PID variables comme ton code
  long _lastTicksG = 0;
  long _lastTicksD = 0;
  long _erreurI = 0;
  unsigned long _lastPID = 0;
  unsigned long _lastDebug = 0;
};
