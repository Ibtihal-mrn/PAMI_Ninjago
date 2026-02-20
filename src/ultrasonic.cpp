#include "../lib/ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig, uint8_t echo, float seuilCm)
: _trig(trig), _echo(echo), _seuil(seuilCm) {}

void Ultrasonic::begin() {
  pinMode(_trig, OUTPUT);
  pinMode(_echo, INPUT);
}

float Ultrasonic::mesurerDistance() {
  digitalWrite(_trig, LOW);
  delayMicroseconds(2);

  digitalWrite(_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trig, LOW);

  long duree = pulseIn(_echo, HIGH, 20000);
  return duree * 0.034f / 2.0f;
}
