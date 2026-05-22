#pragma once
#include <vector>
#include <utility>

class Engine {
private:
    float idleRPM = 800.0f;
    float maxRPM = 7500.0f;
    std::vector<std::pair<float, float>> torqueCurve;

public:
    // 建構子：負責初始化引擎參數
    Engine();

    // Getters：唯讀屬性，加上 const 確保不會修改到內部資料
    float getIdleRPM() const;
    float getMaxRPM() const;

    // 核心功能：根據當前轉速與油門計算輸出扭力
    float getTorque(float currentRPM, float throttle) const;
};
