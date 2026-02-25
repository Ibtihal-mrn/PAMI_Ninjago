#include "../lib/teamSwitch.h"

TeamSwitch::TeamSwitch(uint8_t pin)
: _pin(pin) {}

void TeamSwitch::begin() {
    pinMode(_pin, INPUT_PULLUP);
}

Team TeamSwitch::readTeam() const {
    // Switch fermé (LOW) -> équipe B par exemple
    if (digitalRead(_pin) == LOW) {
        return Team::B;
    } else {
        return Team::A;
    }
}