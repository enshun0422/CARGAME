#include "Gearbox.h"

// 建構子：初始化變速箱硬體參數與預設狀態
Gearbox::Gearbox() {
    // 檔位齒比：索引 0=倒檔(R), 1=空檔(N), 2=一檔, 3=二檔...
    gearRatios = { -2.50f, 0.0f, 2.35f, 1.70f, 1.33f, 1.08f, 0.90f, 0.78f };
    this->finalDrive = 3.15f;
    this->efficiency = 0.95f;
    this->currentGear = 2; // 啟動時預設入 1 檔
}

Gearbox::Gearbox(const std::array<float, 8>& ratios, float finalDrive, float efficiency) {
	this->gearRatios = ratios;
	this->finalDrive = finalDrive;
	this->efficiency = efficiency;
	this->currentGear = 2; // 啟動時預設入 1 檔
}

// --- Getters ---
float Gearbox::getCurrentRatio() const {
    if (currentGear < 0 || currentGear >= static_cast<int>(gearRatios.size())) {
        return 0.0f;
    }
    return gearRatios[currentGear];
}

float Gearbox::getFinalDrive() const {
    return this->finalDrive;
}

float Gearbox::getEfficiency() const {
    return this->efficiency;
}

int Gearbox::getCurrentGear() const {
    if (currentGear < 0 || currentGear >= static_cast<int>(gearRatios.size()))
        return -1;

    return currentGear;
}

// --- 換檔邏輯 (具備安全保護) ---
void Gearbox::shiftUp() {
    if (currentGear < static_cast<int>(gearRatios.size()) - 1) {
        currentGear++;
    }
}

void Gearbox::shiftDown() {
    // 確保不會低於倒檔 (索引 0)
    if (this->currentGear > 0) {
        this->currentGear--;
    }
}

bool Gearbox::setGear(int targetGear) {
    if (targetGear >= 0 && targetGear < static_cast<int>(gearRatios.size())) {
        currentGear = targetGear;
        return true;
    }
    return false;
}
