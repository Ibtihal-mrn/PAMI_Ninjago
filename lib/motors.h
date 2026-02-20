#pragma once
#include <Arduino.h>
#include <Adafruit_MotorShield.h>

class Motors {
public:
  Motors(Adafruit_MotorShield& shield, uint8_t portD, uint8_t portG);

  bool begin(); // appelle shield.begin()
  void stop();

  void runForward();      // chez toi: avancer = BACKWARD
  void runTurnLeft();     // D FORWARD, G BACKWARD

  void setSpeed(int speedD, int speedG);

private:
  Adafruit_MotorShield& _shield;
  Adafruit_DCMotor* _motorD;
  Adafruit_DCMotor* _motorG;
};
