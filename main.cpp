#include <iostream>
#include <iomanip>
#include "Vehicle.h"

int main() {
    Vehicle myCar;
    float dt = 0.016f;    // 模擬 60 FPS 的物理更新頻率
    float timer = 0.0f;
    float throttle = 0.0f;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Time(s) | Throttle | KPH    | SlipRatio | WheelVel(m/s)" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    for (int i = 0; i < 300; ++i) { // 模擬 5 秒鐘 (300 * 0.016)
        timer = i * dt;

        // 測試場景邏輯
        if (timer < 0.5f) {
            throttle = 0.0f; // 靜止
        }
        else if (timer < 3.5f) {
            throttle = 1.0f; // 全油門加速
        }
        else {
            throttle = 0.0f; // 放開油門滑行
        }

        // 執行車輛物理更新
        myCar.applyThrottle(throttle, dt);

        // 每隔 0.2 秒輸出一次數據以便觀察
        if (i % 12 == 0) {
           std::cout<< timer << "    | " 
                     << throttle << "      | " 
                     << myCar.getKPH() << " | " 
                     << myCar.wheels[2].getSlipRatio() << "     | " 
			   << myCar.wheels[2].getTireVelocity() << std::endl;
        }
    }
        

    return 0;
}