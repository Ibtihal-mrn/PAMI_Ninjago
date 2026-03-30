// #include "../lib/fsm.h"

// namespace
// {
//     constexpr long TICKS_5CM = 30;
//     constexpr long TICKS_15CM = 54;
//     constexpr long TICKS_90DEG = 17;
//     constexpr long TICKS_180DEG = 37;
// } // namespace

// Fsm::Fsm(Robot &robot) : _robot(robot) {}

// void Fsm::begin(Team team)
// {
//     _team = team;

//     if (_team == Team::A)
//     {
//         _state = FsmState::TeamA_Start;
//         Serial.println("Team A selected");
//     }
//     else
//     {
//         _state = FsmState::TeamB_Start;
//         Serial.println("Team B selected");
//     }
// }

// void Fsm::update()
// {
//     switch (_state)
//     {
//     case FsmState::Idle:
//         break;

//     case FsmState::TeamA_Start:
//         Serial.println("Start sequence Team A");
//         _state = FsmState::TeamA_Forward1;
//         break;

//     case FsmState::TeamA_Forward1:
//         Serial.println("Forward 5 cm");
//         _robot.avancer_ticks(TICKS_5CM);
//         _state = FsmState::TeamA_Turn1;
//         break;

//     case FsmState::TeamA_Turn1:
//         Serial.println("Turn left 90 deg");
//         _robot.tourner_gauche_ticks(TICKS_90DEG);
//         _state = FsmState::TeamA_Forward2;
//         break;

//     case FsmState::TeamA_Forward2:
//         Serial.println("Forward 15 cm");
//         _robot.avancer_ticks(TICKS_15CM);
//         _state = FsmState::TeamA_TurnBack1;
//         break;

//     case FsmState::TeamA_TurnBack1:
//         Serial.println("Turn 180 deg");
//         _robot.tourner_gauche_ticks(TICKS_180DEG);
//         _state = FsmState::TeamA_Forward3;
//         break;

//     case FsmState::TeamA_Forward3:
//         Serial.println("Forward calibrated segment 1");
//         _robot.avancer_ticks(55);
//         _state = FsmState::TeamA_Turn2;
//         break;

//     case FsmState::TeamA_Turn2:
//         Serial.println("Turn left 90 deg");
//         _robot.tourner_gauche_ticks(TICKS_90DEG);
//         _state = FsmState::TeamA_Forward4;
//         break;

//     case FsmState::TeamA_Forward4:
//         Serial.println("Forward calibrated segment 2");
//         _robot.avancer_ticks(64);
//         _state = FsmState::TeamA_Turn3;
//         break;

//     case FsmState::TeamA_Turn3:
//         Serial.println("Turn left 90 deg");
//         _robot.tourner_gauche_ticks(TICKS_90DEG);
//         _state = FsmState::TeamA_Forward5;
//         break;

//     case FsmState::TeamA_Forward5:
//         Serial.println("Forward 5 cm plus margin");
//         _robot.avancer_ticks(TICKS_5CM + 20);
//         _state = FsmState::TeamA_TurnBack2;
//         break;

//     case FsmState::TeamA_TurnBack2:
//         Serial.println("Final 180 deg turn");
//         _robot.tourner_gauche_ticks(TICKS_180DEG);
//         _state = FsmState::TeamA_Done;
//         break;

//     case FsmState::TeamA_Done:
//         Serial.println("Team A sequence finished");
//         _state = FsmState::Finished;
//         break;

//     case FsmState::TeamB_Start:
//         Serial.println("Start sequence Team B");
//         Serial.println("Team B sequence not implemented yet");
//         _state = FsmState::TeamB_Done;
//         break;

//     case FsmState::TeamB_Done:
//         Serial.println("Team B sequence finished");
//         _state = FsmState::Finished;
//         break;

//     case FsmState::Finished:
//         break;
//     }
// }

// bool Fsm::isFinished() const
// {
//     return _state == FsmState::Finished;
// }

// FsmState Fsm::state() const
// {
//     return _state;
// }
