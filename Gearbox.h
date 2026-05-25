#pragma once
#include <array>

class Gearbox {
private:
    std::array<float, 8> gearRatios{};
    float finalDrive;
    float efficiency;
    int currentGear; // 內部狀態：0=倒檔, 1=空檔, 2=一檔...

public:
    Gearbox();

    // Getters (唯讀，確保 const 正確性)
    float getCurrentRatio() const;
    float getFinalDrive() const;
    float getEfficiency() const;
    int getCurrentGear() const;

    // 行為與狀態變更 (封裝換檔邏輯)
    void shiftUp();
    void shiftDown();
    bool setGear(int targetGear); // 回傳是否進檔成功
};
