#pragma once
#include "common.h"

class MotionModel {
public:
    MotionModel(double max_speed, double acceleration, double deceleration)
        : v_max(max_speed), a(acceleration), d(deceleration) {}

    // Compute the time to reach maximum speed from an initial speed
    double timeToMaxSpeed(double initial_speed) const;

    // Compute the distance needed to reach maximum speed from an initial speed
    double distanceToMaxSpeed(double initial_speed) const;

    // Compute the time to stop from the current speed
    double timeToStop(double current_speed) const;

    // Compute the distance needed to stop from the current speed
    double distanceToStop(double current_speed) const;

    // Compute sweeping time based on distance traveled
    double computeSweepingTime(double initial_speed, double distance);

    MotionModel(): v_max(0), a(0), d(0) {}

private:
    double v_max;  // Maximum speed
    double a;      // Acceleration
    double d;      // Deceleration
};