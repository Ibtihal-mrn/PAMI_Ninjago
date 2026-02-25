#pragma once
#include <Arduino.h>

class StartSwitch {
public:
    explicit StartSwitch(uint8_t pin);

    void begin();
    void waitForStart();
    bool isInserted() const;   // optionnel si tu veux juste tester l'état

private:
    uint8_t _pin;
};