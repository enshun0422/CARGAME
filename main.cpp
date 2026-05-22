#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <string>
#include "Vehicle.h"

int main() {
    // 初始化 BMW M4 GT3 EVO 車輛物件
    Vehicle myCar;

    // 設定模擬參數
    float dt = 0.016f;         // 物理幀率 60 FPS
    float totalTime = 0.0f;    // 累積模擬時間
    int printInterval = 10;    // 每 10 幀 (約 0.16 秒) 輸出一次數據

    // 格式化輸出主標頭
    std::cout << "==========================================================================================================================================================================\n";
    std::cout << "                                                    BMW M4 GT3 EVO 物理引擎遙測數據主控台 (Console Telemetry)                                             \n";
    std::cout << "==========================================================================================================================================================================\n";

    // 符合需求之標準欄位列印
    std::cout << std::setw(6) << "frame" << " | "
        << std::setw(8) << "status" << " | "
        << std::setw(8) << "seconds" << " | "
        << std::setw(5) << "Gear" << " | "
        << std::setw(7) << "RPM" << " | "
        << std::setw(14) << "car_vel(KPH)" << " | "
        << std::setw(15) << "tire_vel(KPH)" << " | "
        << std::setw(10) << "slip_ratio" << " | "
        << std::setw(11) << "slip_angle" << " | "
        << std::setw(13) << "suspension FL" << " | "
        << std::setw(13) << "suspension FR" << " | "
        << std::setw(13) << "suspensionRL" << " | "
        << std::setw(13) << "suspensionRR" << " | "
        << std::setw(11) << "steer_angle" << " | "
        << std::setw(14) << "World_Pos(X,Y)" << "\n";
    std::cout << "--------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";

    int totalFrames = 39.0f / dt;

    for (int frame = 0; frame <= totalFrames; frame++) {
        // ==========================================
        // 1. 駕駛行為模擬 (只負責踩踏板與轉方向盤)
        // ==========================================
        float throttle = 0.0f;
        float brakeForce = 0.0f;
        float steerInput = 0.0f;
        std::string stageStatus = "";

        if (totalTime < 19.0f) {
            throttle = 1.0f; stageStatus = "[ACCEL]";
        }
        else if (totalTime < 21.0f) {
            brakeForce = 15000.0f; stageStatus = "[BRAKE]";
        }
        else if (totalTime < 22.0f) {
            throttle = 0.2f;
            steerInput = 0.05f;  // 【修正】從 0.35 改為 0.05 (約 3 度)
            stageStatus = "[TURN_L]";
        }
        else if (totalTime < 23.0f) {
            throttle = 0.2f;
            steerInput = -0.05f; // 【修正】從 -0.35 改為 -0.05 (約 3 度)
            stageStatus = "[TURN_R]";
        }
        else if (totalTime < 29.0f) {
            stageStatus = "[COAST]"; // 無油門無煞車滑行
        }
        else if (totalTime < 34.0f) {
            throttle = 1.0f; stageStatus = "[ACCEL]";
        }
        else if (totalTime <= 39.0f) {
            brakeForce = 15000.0f; stageStatus = "[BRAKE]";
        }

        // ==========================================
        // 2. 變速箱電腦 (TCU) 獨立邏輯
        // ==========================================
        int currentGearIdx = myCar.gearbox.getCurrentGear();
        float currentSpeed = myCar.getKPH();

        // [升檔邏輯]：只要有踩油門，且轉速/車速達到門檻就升檔
        if (throttle > 0.1f) {
            if (currentSpeed > 270.0f && currentGearIdx == 6) myCar.gearbox.shiftUp(); // 5升6
            else if (currentSpeed > 230.0f && currentGearIdx == 5) myCar.gearbox.shiftUp(); // 4升5
            else if (currentSpeed > 180.0f && currentGearIdx == 4) myCar.gearbox.shiftUp(); // 3升4
            else if (currentSpeed > 130.0f && currentGearIdx == 3) myCar.gearbox.shiftUp(); // 2升3
            else if (currentSpeed > 80.0f && currentGearIdx == 2) myCar.gearbox.shiftUp(); // 1升2
        }

        // [降檔邏輯]：只要沒踩油門（煞車或滑行），積極維持高轉速降檔
        // 注意：這裡門檻大幅提高，模擬 GT3 賽車的激進退檔
        if (throttle <= 0.1f) {
            if (currentSpeed < 260.0f && currentGearIdx == 7) myCar.gearbox.shiftDown(); // 6退5
            else if (currentSpeed < 220.0f && currentGearIdx == 6) myCar.gearbox.shiftDown(); // 5退4
            else if (currentSpeed < 170.0f && currentGearIdx == 5) myCar.gearbox.shiftDown(); // 4退3
            else if (currentSpeed < 120.0f && currentGearIdx == 4) myCar.gearbox.shiftDown(); // 3退2
            else if (currentSpeed < 70.0f && currentGearIdx == 3) myCar.gearbox.shiftDown(); // 2退1
        }

        // ==========================================
        // 3. 傳遞物理訊號與推進物理幀
        // ==========================================
        myCar.setSteeringAngle(steerInput);
        myCar.update(throttle, brakeForce, dt);

        totalTime += dt;

        // 定時抽樣輸出遙測數據
        if (frame % printInterval == 0) {

            float tireKPH = myCar.tires[2].tire.getTireVelocity() * 3.6f;

            // 換算縱向滑移率
            float refSpeed = std::max(std::abs(myCar.getForwardVelocity()), 1.0f);
            float slipRatioVal = (myCar.tires[2].tire.getTireVelocity() - myCar.getForwardVelocity()) / refSpeed;

            // 換算前輪側向滑移角
            float rx_front = 1.3f;
            float wheelVy = myCar.getLateralVelocity() + myCar.getYawRate() * rx_front;
            float slipAngleDeg = (std::atan2(wheelVy, refSpeed) - steerInput) * (180.0f / PI);

            // 計算避震器正向力 (模擬防傾桿行程差異)
            float staticW = (myCar.getTotalMass() * 9.81f) / 4.0f;
            float currentFrontDF = myCar.aero.calculateFrontDownforce(myCar.getForwardVelocity());
            float currentRearDF = myCar.aero.calculateRearDownforce(myCar.getForwardVelocity());

            float compression_mock[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
            if (steerInput < 0.0f) { // 左轉：車身向右傾 (外側壓，內側伸)
                compression_mock[0] = 0.02f;
                compression_mock[1] = -0.02f;
                compression_mock[2] = 0.02f;
                compression_mock[3] = -0.02f;
            }
            else if (steerInput > 0.0f) { // 右轉：車身向左傾
                compression_mock[0] = -0.02f;
                compression_mock[1] = 0.02f;
                compression_mock[2] = -0.02f;
                compression_mock[3] = 0.02f;
            }

            float frontARB = (compression_mock[0] - compression_mock[1]) * myCar.getAntiRollBarStiffnessFront();
            float rearARB = (compression_mock[2] - compression_mock[3]) * myCar.getAntiRollBarStiffnessRear();

            float suspFL = std::max(0.0f, staticW + (currentFrontDF / 2.0f) + frontARB);
            float suspFR = std::max(0.0f, staticW + (currentFrontDF / 2.0f) - frontARB);
            float suspRL = std::max(0.0f, staticW + (currentRearDF / 2.0f) + rearARB);
            float suspRR = std::max(0.0f, staticW + (currentRearDF / 2.0f) - rearARB);

            // 5. 計算顯示用之引擎 RPM (重寫離合器接合邏輯)
            float currentGearRatio = myCar.gearbox.getCurrentRatio();

            // 【儀表板同步修正】：一樣從後輪抓取真實輪速
            float drivenWheelRad = std::abs(myCar.tires[2].tire.getAngularVel());
            float wheelRPM = drivenWheelRad * (60.0f / TWO_PI);
            float baseEngineRPM = wheelRPM * std::abs(currentGearRatio) * myCar.gearbox.getFinalDrive();

            float engineRPM = baseEngineRPM;

            // 離合器模擬 (維持原樣)
            if (std::abs(myCar.getKPH()) < 60.0f && throttle > 0.1f) {
                float launchRPM = myCar.engine.getMaxRPM() * 0.6f;
                float targetLaunchRPM = myCar.engine.getIdleRPM() + throttle * (launchRPM - myCar.engine.getIdleRPM());
                float clutchSlip = 1.0f - (std::abs(myCar.getKPH()) / 60.0f);
                clutchSlip = std::clamp(clutchSlip, 0.0f, 1.0f);
                engineRPM = (targetLaunchRPM * clutchSlip) + (baseEngineRPM * (1.0f - clutchSlip));
            }

            // 【儀表板同步修正】：解除 clamp，讓你親眼在 Console 看到轉速撞到紅線 7600 轉彈跳的快感！
            engineRPM = std::max(engineRPM, myCar.engine.getIdleRPM());

            std::cout << std::setw(6) << frame << " | "
                << std::setw(8) << stageStatus << " | "
                << std::setw(7) << std::fixed << std::setprecision(2) << totalTime << "s | "
                << std::setw(4) << (myCar.gearbox.getCurrentGear() - 1) << "檔 | "
                << std::setw(7) << std::setprecision(0) << engineRPM << " | "
                << std::setw(14) << std::setprecision(2) << myCar.getKPH() << " | "
                << std::setw(15) << tireKPH << " | "
                << std::setw(10) << std::setprecision(4) << slipRatioVal << " | "
                << std::setw(11) << std::setprecision(2) << slipAngleDeg << " | "
                << std::setw(13) << std::setprecision(0) << suspFL << " | "
                << std::setw(13) << std::setprecision(0) << suspFR << " | "
                << std::setw(13) << std::setprecision(0) << suspRL << " | "
                << std::setw(13) << std::setprecision(0) << suspRR << " | "
                << std::setw(11) << std::setprecision(2) << steerInput << " | "
                << "(" << std::setprecision(1) << myCar.getWorldX() << "," << myCar.getWorldY() << ")\n";
        }
    }

    return 0;
}