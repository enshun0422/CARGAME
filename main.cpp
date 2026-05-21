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
        float throttle = 0.0f;
        float brakeForce = 0.0f;
        float steerInput = 0.0f;
        std::string stageStatus = "";

        // 【測試情境排程：0 ~ 39 秒】
        if (totalTime < 19.0f) {
            // 0~19秒：直線全力加速
            throttle = 1.0f;
            brakeForce = 0.0f;
            steerInput = 0.0f;
            stageStatus = "[ACCEL]";

            // 自動升檔邏輯
            if (myCar.getKPH() > 80.0f && myCar.gearbox.getCurrentGear() == 2) myCar.gearbox.shiftUp();
            if (myCar.getKPH() > 130.0f && myCar.gearbox.getCurrentGear() == 3) myCar.gearbox.shiftUp();
            if (myCar.getKPH() > 180.0f && myCar.gearbox.getCurrentGear() == 4) myCar.gearbox.shiftUp();
            if (myCar.getKPH() > 230.0f && myCar.gearbox.getCurrentGear() == 5) myCar.gearbox.shiftUp();
        }
        else if (totalTime < 21.0f) {
            // 19~21秒：直線重煞車
            throttle = 0.0f;
            brakeForce = 15000.0f;
            steerInput = 0.0f;
            stageStatus = "[BRAKE]";

            // 自動退檔邏輯
            if (myCar.getKPH() < 200.0f && myCar.gearbox.getCurrentGear() == 6) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 150.0f && myCar.gearbox.getCurrentGear() == 5) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 100.0f && myCar.gearbox.getCurrentGear() == 4) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 60.0f && myCar.gearbox.getCurrentGear() == 3) myCar.gearbox.shiftDown();
        }
        else if (totalTime < 22.0f) {
            // 21~22秒：左轉
            throttle = 0.2f;
            brakeForce = 0.0f;
            steerInput = -0.35f;
            stageStatus = "[TURN_L]";
        }
        else if (totalTime < 23.0f) {
            // 22~23秒：右轉
            throttle = 0.2f;
            brakeForce = 0.0f;
            steerInput = 0.35f;
            stageStatus = "[TURN_R]";
        }
        else if (totalTime < 29.0f) {
            // 23~29秒：出彎再度加速
            throttle = 1.0f;
            brakeForce = 0.0f;
            steerInput = 0.0f;
            stageStatus = "[ACCEL]";
            if (myCar.getKPH() > 80.0f && myCar.gearbox.getCurrentGear() == 2) myCar.gearbox.shiftUp();
            if (myCar.getKPH() > 130.0f && myCar.gearbox.getCurrentGear() == 3) myCar.gearbox.shiftUp();
            if (myCar.getKPH() > 180.0f && myCar.gearbox.getCurrentGear() == 4) myCar.gearbox.shiftUp();
            if (myCar.getKPH() > 230.0f && myCar.gearbox.getCurrentGear() == 5) myCar.gearbox.shiftUp();
        }
        else if (totalTime < 34.0f) {
            // 29~34秒：無油門無煞車滑行
            throttle = 0.0f;
            brakeForce = 0.0f;
            steerInput = 0.0f;
            stageStatus = "[COAST]";
            if (myCar.getKPH() < 200.0f && myCar.gearbox.getCurrentGear() == 6) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 150.0f && myCar.gearbox.getCurrentGear() == 5) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 100.0f && myCar.gearbox.getCurrentGear() == 4) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 60.0f && myCar.gearbox.getCurrentGear() == 3) myCar.gearbox.shiftDown();
        }
        else if (totalTime <= 39.0f) {
            // 34~39秒：最終重煞車至停止
            throttle = 0.0f;
            brakeForce = 15000.0f;
            steerInput = 0.0f;
            stageStatus = "[BRAKE]";
            if (myCar.getKPH() < 200.0f && myCar.gearbox.getCurrentGear() == 6) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 150.0f && myCar.gearbox.getCurrentGear() == 5) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 100.0f && myCar.gearbox.getCurrentGear() == 4) myCar.gearbox.shiftDown();
            if (myCar.getKPH() < 60.0f && myCar.gearbox.getCurrentGear() == 3) myCar.gearbox.shiftDown();
        }

        // 傳遞控制訊號並推進物理幀
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
            float wheelRPM = (myCar.getForwardVelocity() / (2.0f * PI * myCar.tires[0].tire.getRadius())) * 60.0f;
            float baseEngineRPM = std::abs(wheelRPM * currentGearRatio * myCar.gearbox.getFinalDrive());

            float engineRPM = baseEngineRPM;

            // 離合器模擬：在低速起步階段允許轉速攀升
            if (myCar.getKPH() < 60.0f && throttle > 0.1f) {
                // 1. 計算全油門時的目標彈射轉速 (約斷油轉速的 60%)
                float launchRPM = myCar.engine.getMaxRPM() * 0.6f;

                // 2. 根據當前油門深度，決定實際的拉轉目標
                float targetLaunchRPM = myCar.engine.getIdleRPM() + throttle * (launchRPM - myCar.engine.getIdleRPM());

                // 3. 計算離合器接合比例 (Slip Ratio)
                // 假設車速達到 60 KPH 時離合器完全接合 (比例為 0)
                // 車速為 0 時，離合器滑動最嚴重 (比例為 1)
                float clutchSlip = 1.0f - (myCar.getKPH() / 60.0f);
                clutchSlip = std::clamp(clutchSlip, 0.0f, 1.0f);

                // 4. 將基礎轉速與拉轉轉速進行混合 (Interpolation)
                // 當 clutchSlip 為 1 (靜止) 時，引擎完全聽油門的 (targetLaunchRPM)
                // 當 clutchSlip 為 0 (超過 60KPH) 時，引擎完全綁定輪速 (baseEngineRPM)
                engineRPM = (targetLaunchRPM * clutchSlip) + (baseEngineRPM * (1.0f - clutchSlip));
            }

            // 確保轉速不會低於怠速或超過紅線
            engineRPM = std::clamp(engineRPM, myCar.engine.getIdleRPM(), myCar.engine.getMaxRPM());

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