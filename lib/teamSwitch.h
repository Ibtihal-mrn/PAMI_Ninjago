#pragma once
#include <Arduino.h>

enum class Team {
    A,
    B
};

class TeamSwitch {
public:
    explicit TeamSwitch(uint8_t pin);

    void begin();
    Team readTeam() const;

private:
    uint8_t _pin;
};