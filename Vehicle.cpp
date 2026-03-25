#include "Vehicle.h"
#include "Wheel.h"
#include <iostream>
#include <cmath>



void Vehicle::applyThrottle(float throttle, float dt) {
    float engineTorque = throttle * 500.0f;
    float totalFx = 0.0f;

    // 1. 修正索引從 0 開始，到 3 結束
    for (int i = 0; i < 4; i++) {
        wheels[i].calculslipRatio(forwardVelocity);

        // 取得該輪摩擦力
        float tireFx = wheels[i].updateSlipState();
        totalFx += tireFx;

        // 2. 只有後輪 (2, 3) 給動力，前輪 (0, 1) 給 0
        float torqueToApply = (i >= 2) ? (engineTorque / 2.0f) : 0.0f;
        wheels[i].update(torqueToApply, dt);
    }

    // 3. 額外加入空氣阻力 (簡單模擬)
    float airResistance = 0.5f * forwardVelocity * forwardVelocity; // 隨便給個係數
    float netForce = totalFx - airResistance;

    // 4. 更新加速度與速度
    float acceleration = netForce / totalMass;
    forwardVelocity += acceleration * dt;

    if (forwardVelocity < 0.0f) forwardVelocity = 0.0f;
}

float Vehicle::getKPH() {
    return forwardVelocity * 3.6f; // m/s 轉 km/h
}