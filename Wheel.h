#pragma once

class Wheel {
public:
    float radius = 0.33f;    // 18吋胎半徑約 0.33公尺
    float angularVel = 0.0f; // 角速度 (rad/s)
    float inertia = 1.2f;    // 輪胎轉動慣量

    void update(float torque, float dt) {
        // 公式：角加速度 = 扭矩 / 轉動慣量
        float angularAccel = torque / inertia;
        angularVel += angularAccel * dt;
    }

    float getVelocity() { return angularVel * radius; }
};
