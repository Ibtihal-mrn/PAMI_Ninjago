#include "../lib/ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig, uint8_t echo, uint8_t trig1, uint8_t echo1, float seuilCm)
: _trig(trig), _echo(echo), _trig1(trig1), _echo1(echo1), _seuil(seuilCm) {}

void Ultrasonic::begin() {
  pinMode(_trig, OUTPUT);
  pinMode(_echo, INPUT);
  pinMode(_trig1, OUTPUT);
  pinMode(_echo1, INPUT);
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
