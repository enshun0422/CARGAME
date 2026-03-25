#pragma once
#include <vector>
#include "Wheel.h"

class Vehicle {
public:
    Wheel wheels[4];
	float forwardVelocity = 0.0f; // 前進速度 (m/s)
    float totalMass = 1500.0f; // 1.5 噸

    void applyThrottle(float throttle, float dt);
    float getKPH();
};
