#include "../lib/encoders.h"

Encoders* Encoders::_self = nullptr;

Encoders::Encoders(uint8_t pinG, uint8_t pinD) : _pinG(pinG), _pinD(pinD) {}

void Encoders::begin() {
  pinMode(_pinG, INPUT_PULLUP);
  pinMode(_pinD, INPUT_PULLUP);

  attach(this);
  attachInterrupt(digitalPinToInterrupt(_pinG), Encoders::ISR_G, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinD), Encoders::ISR_D, CHANGE);
}

void Encoders::reset() {
  noInterrupts();
  _ticksG = 0;
  _ticksD = 0;
  interrupts();
}

void Encoders::attach(Encoders* instance) {
  _self = instance;
}

void Encoders::ISR_G() { if (_self) _self->isrG(); }
void Encoders::ISR_D() { if (_self) _self->isrD(); }

void Encoders::isrG() {
  static unsigned long last = 0;
  unsigned long now = micros();
  if (now - last > 300) {
    _ticksG++;
    last = now;
  }
}

void Encoders::isrD() {
  static unsigned long last = 0;
  unsigned long now = micros();
  if (now - last > 300) {
    _ticksD++;
    last = now;
  }
}
