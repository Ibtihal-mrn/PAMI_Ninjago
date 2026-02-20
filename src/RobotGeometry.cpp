#include "../lib/RobotGeometry.h"

static constexpr float PI_F = 3.1415926f;

RobotGeometry::RobotGeometry(float wheelDiamCm,
                             float wheelBaseCm,
                             float ticksPerRevLeft,
                             float ticksPerRevRight)
: _wheelDiamCm(wheelDiamCm),
  _wheelBaseCm(wheelBaseCm)
{
    _ticksPerRevAvg = 0.5f * (ticksPerRevLeft + ticksPerRevRight);
}

long RobotGeometry::cmToTicks(float distanceCm) const {
    float circumference = PI_F * _wheelDiamCm;
    float ticks = _kDrive * (distanceCm / circumference) * _ticksPerRevAvg;
    return lroundf(ticks);
}

long RobotGeometry::degToTicks(float deg) const {
    float arc = PI_F * _wheelBaseCm * (deg / 360.0f);
    float circumference = PI_F * _wheelDiamCm;
    float ticks = _kTurn * (arc / circumference) * _ticksPerRevAvg;
    return lroundf(ticks);
}
