#pragma once
#include <Arduino.h>
#include "ultrasonic.h"
#include "emergencyButton.h"
#include "life.h"
#include "motors.h"

class Safety {
public:
  Safety(Ultrasonic& us, EmergencyButton& btn, Life& life, Motors& motors);

  // return true => stop immédiat (urgence ou fin de vie)
  bool check();

private:
  Ultrasonic& _us;
  EmergencyButton& _btn;
  Life& _life;
  Motors& _motors;

  bool _obstaclePresent = false;
};
