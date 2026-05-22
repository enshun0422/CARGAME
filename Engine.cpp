#include "Engine.h"

// 建構子實作：設定預設的引擎參數與扭力曲線
Engine::Engine() {
    this->idleRPM = 1000.0f;
    this->maxRPM = 7500.0f;

    // 扭力曲線對照表 (RPM, Torque in Nm)
    this->torqueCurve = {
        {1000, 250.0f},
        {2500, 500.0f},
        {4000, 680.0f},
        {5000, 700.0f}, // 扭力峰值
        {6000, 650.0f},
        {7000, 550.0f},
        {7500, 450.0f}
    };
}

// Getters 實作
float Engine::getIdleRPM() const {
    return this->idleRPM;
}

float Engine::getMaxRPM() const {
    return this->maxRPM;
}

// 計算扭力實作
float Engine::getTorque(float currentRPM, float throttle) const {
    // 轉速低於曲線起點，輸出怠速扭力
    if (currentRPM < torqueCurve.front().first) {
        return torqueCurve.front().second * throttle;
    }

    // 【修正物理】超過紅線不應該給 0，而是產生極大的機械阻力(引擎煞車)
    if (currentRPM > torqueCurve.back().first) {
        // 隨著超轉越嚴重，負扭力越大 (模擬活塞壓縮阻力)
        float overRevRatio = (currentRPM - torqueCurve.back().first) / 1000.0f;
        return -300.0f - (200.0f * overRevRatio);
    }

    // 尋找對應的區間並進行線性插值 (Lerp)
    for (size_t i = 0; i < torqueCurve.size() - 1; i++) {
        if (currentRPM >= torqueCurve[i].first && currentRPM <= torqueCurve[i + 1].first) {
            float t = (currentRPM - torqueCurve[i].first) / (torqueCurve[i + 1].first - torqueCurve[i].first);
            float maxTorqueAtRPM = torqueCurve[i].second + t * (torqueCurve[i + 1].second - torqueCurve[i].second);
            return maxTorqueAtRPM * throttle;
        }
    }

    return 0.0f;
}