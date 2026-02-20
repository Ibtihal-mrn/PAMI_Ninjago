#pragma once
#include <Arduino.h>

class RobotGeometry {
public:
    RobotGeometry(float wheelDiamCm,
                  float wheelBaseCm,
                  float ticksPerRevLeft,
                  float ticksPerRevRight);

    long cmToTicks(float distanceCm) const;
    long degToTicks(float deg) const;

    void setDriveCalibration(float k) { _kDrive = k; }
    void setTurnCalibration(float k)  { _kTurn  = k; }

private:
    float _wheelDiamCm;
    float _wheelBaseCm;
    float _ticksPerRevAvg;

    float _kDrive = 1.0f;
    float _kTurn  = 1.0f;
};
