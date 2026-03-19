#pragma once

#include <Arduino.h>
#include "robot.h"
#include "teamSwitch.h"

enum class FsmState
{
    Idle,
    TeamA_Start,
    TeamA_Forward1,
    TeamA_Turn1,
    TeamA_Forward2,
    TeamA_TurnBack1,
    TeamA_Forward3,
    TeamA_Turn2,
    TeamA_Forward4,
    TeamA_Turn3,
    TeamA_Forward5,
    TeamA_TurnBack2,
    TeamA_Done,
    TeamB_Start,
    TeamB_Done,
    Finished
};

class Fsm
{
public:
    explicit Fsm(Robot &robot);

    void begin(Team team);
    void update();

    bool isFinished() const;
    FsmState state() const;

private:
    Robot &_robot;
    Team _team = Team::A;
    FsmState _state = FsmState::Idle;
};
