#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "Wheel.h"
#include "TireDynamics.h"

class Vehicle {
public:
    TireDynamics tires[4];
	float forwardVelocity = 0.0f; // 前進速度 (m/s)
    float totalMass = 1300.0f; // 噸
    
	void update(float driveTorque, float brakeForce, float dt);
    float getKPH();
};
