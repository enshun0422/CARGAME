#include <iostream>
#include <Windows.h>
#include "Vehicle.h"

int main() {
    SetConsoleOutputCP(65001);

    Vehicle car;
    float dt = 0.01f; // 每一幀模擬 0.01 秒

    std::cout << "--- Starting Vehicle Physics Simulation ---" << std::endl;

    for (int i = 0; i < 100; ++i) {
        car.applyThrottle(1.0f, dt); // 踩油門
        if (i % 10 == 0) {
            std::cout << "Time: " << i * dt << "s, speed: " << car.getKPH() << " km/h" << std::endl;
        }
    }

    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Simulation finished. Press Enter to exit...";
    std::cin.get();
    return 0;
    //Vehicle car;
    //float dt = 0.016f; // 假設 60 FPS

    //std::cout << "Accelerate starting..." << std::endl;
    //for (int i = 0; i < 100; i++) {
    //    car.applyThrottle(1.0f, dt); // 油門踩到底
    //    if (i % 10 == 0) {
    //        std::cout << "Corrent Speed: " << car.getKPH() << " km/h" << std::endl;
    //    }
    //}
    //return 0;
}