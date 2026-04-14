#include "../lib/startSwitch.h"

StartSwitch::StartSwitch(uint8_t pin)
: _pin(pin) {}

void StartSwitch::begin() {
    pinMode(_pin, INPUT_PULLUP);
}

bool StartSwitch::isInserted() const {
    // Comme ton sketch: avec INPUT_PULLUP, tirette INSÉRÉE = circuit ouvert = HIGH.
    // Tirette RETIRÉE = broche tirée vers GND = LOW.
    return digitalRead(_pin) == HIGH;
}

void StartSwitch::waitForStart() {
    Serial.println("\xF0\x9F\x9F\xA1 En attente de la tirette...");

    // Identique à ton sketch: on attend tant que la broche est HIGH.
    while (isInserted())
    {
        delay(10);
    }

    Serial.println("\xF0\x9F\x9A\x80 Tirette retiree -> DEMARRAGE");
    delay(500);
    delay(3000);
}