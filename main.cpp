#include <iostream>
#include <iomanip>
#include <string>
#include "Vehicle.h"

int main() {
    Vehicle myCar;

    // --- 60 FPS 核心設定 ---
    float dt = 1.0f / 60.0f;   // 每幀約 0.01667 秒
    int totalFrames = 10 * 60; // 10秒 * 60幀 = 600 幀
    float currentTime = 0.0f;

    // 印出 CSV 標題列
    std::cout << "Frame,Time(s),State,Throttle(Nm),Brake(N),VehSpeed(km/h),WheelSpeed(km/h),SlipRatio\n";

    for (int i = 0; i < totalFrames; ++i) {
        float throttle = 0.0f;
        float brake = 0.0f;
        std::string stateStr;


        // 10秒狀態
        if (currentTime < 3.0f) {
            // [狀態一：加速] 0 ~ 3 秒
            throttle = 800.0f;
            brake = 0.0f;
            stateStr = "加速";
        }
        else if (currentTime < 6.0f) {
            // [狀態二：滑行] 3 ~ 6 秒
            throttle = 0.0f;
            brake = 0.0f;
            stateStr = "滑行";
        }
        else {
            // [狀態三：煞車] 6 ~ 10 秒
            throttle = 0.0f;
            // 這裡設定 2500N，讓煞車鎖死的過程稍微拉長，你會在數據中看得很清楚
            brake = 2500.0f;
            stateStr = "煞車";
        }


        myCar.update(throttle, brake, dt);

        
        TireDynamics& rearTire = myCar.tires[3]; // 觀察右後輪

        float vehicleSpeedKPH = myCar.getKPH();
        float wheelSpeedKPH = rearTire.wheel.getTireVelocity() * 3.6f;
        float currentSlipRatio = rearTire.calculslipRatio(myCar.forwardVelocity);

        std::cout << std::fixed << std::setprecision(4)
            << std::setw(5) << i << ","
            << std::setw(7) << currentTime << ","
            << std::setw(5) << stateStr << ","
            << std::setw(12) << throttle << ","
            << std::setw(8) << brake << ","
            << std::setw(14) << vehicleSpeedKPH << ","
            << std::setw(16) << wheelSpeedKPH << ","
            << std::setw(9) << currentSlipRatio << "\n";

        currentTime += dt;
    }

    return 0;
}