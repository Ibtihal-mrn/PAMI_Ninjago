#include "../lib/startSwitch.h"

StartSwitch::StartSwitch(uint8_t pin)
: _pin(pin) {}

void StartSwitch::begin() {
    pinMode(_pin, INPUT_PULLUP);
}

bool StartSwitch::isInserted() const {
    // Tirette insérée = contact vers GND = LOW
    return digitalRead(_pin) == LOW;
}

void StartSwitch::waitForStart() {
    Serial.println("En attente de la tirette...");

    // attendre qu'elle soit insérée d'abord
    while (!isInserted()) {
        delay(10);
    }

    // attendre qu'on la retire
    while (isInserted()) {
        delay(10);
    }

    Serial.println("Tirette retiree -> DEMARRAGE !");
    delay(300);
}