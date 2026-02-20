#include "../lib/motors.h"

Motors::Motors(Adafruit_MotorShield& shield, uint8_t portD, uint8_t portG)
: _shield(shield) {
  _motorD = _shield.getMotor(portD);
  _motorG = _shield.getMotor(portG);
}

bool Motors::begin() {
  return _shield.begin();
}

void Motors::stop() {
  _motorG->run(RELEASE);
  _motorD->run(RELEASE);
}

void Motors::runForward() {
  _motorD->run(BACKWARD);
  _motorG->run(BACKWARD);
}

void Motors::runTurnLeft() {
  _motorD->run(FORWARD);
  _motorG->run(BACKWARD);
}

void Motors::setSpeed(int speedD, int speedG) {
  _motorD->setSpeed(constrain(speedD, 0, 255));
  _motorG->setSpeed(constrain(speedG, 0, 255));
}
