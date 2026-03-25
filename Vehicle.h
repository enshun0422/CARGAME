#pragma once
#include <vector>
#include "Wheel.h"

class Vehicle {
public:
    Wheel wheels[4];
    float totalMass = 1500.0f; // 1.5 噸

    void applyThrottle(float throttle, float dt) {
        float engineTorque = throttle * 500.0f; // 假設引擎最大扭矩 500Nm
        // 後輪驅動 (後方兩顆輪子)
        wheels[2].update(engineTorque / 2.0f, dt);
        wheels[3].update(engineTorque / 2.0f, dt);
    }

    float getKPH() {
        return wheels[2].getVelocity() * 3.6f; // m/s 轉 km/h
    }
};
