#include "Gearbox.h"

// 建構子：初始化變速箱硬體參數與預設狀態
Gearbox::Gearbox() {
    // 檔位齒比：索引 0=倒檔(R), 1=空檔(N), 2=一檔, 3=二檔...
    this->gearRatios = { -2.50f, 0.0f, 2.35f, 1.70f, 1.33f, 1.08f, 0.90f, 0.78f };
    this->finalDrive = 3.15f;
    this->efficiency = 0.95f;
    this->currentGear = 2; // 啟動時預設入 1 檔
}

// --- Getters ---
float Gearbox::getCurrentRatio() const {
    return this->gearRatios[this->currentGear];
}

float Gearbox::getFinalDrive() const {
    return this->finalDrive;
}

float Gearbox::getEfficiency() const {
    return this->efficiency;
}

int Gearbox::getCurrentGear() const {
    return this->currentGear;
}

// --- 換檔邏輯 (具備安全保護) ---
void Gearbox::shiftUp() {
    // 確保不會超過最高檔位
    if (this->currentGear < this->gearRatios.size() - 1) {
        this->currentGear++;
    }
}

void Gearbox::shiftDown() {
    // 確保不會低於倒檔 (索引 0)
    if (this->currentGear > 0) {
        this->currentGear--;
    }
}

bool Gearbox::setGear(int targetGear) {
    // 嚴格檢查傳入的檔位是否在合法陣列範圍內
    if (targetGear >= 0 && targetGear < this->gearRatios.size()) {
        this->currentGear = targetGear;
        return true;
    }
    return false; // 防止切入不存在的檔位
}