#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <string>
#include "AMGGT3.h"
#include "Vehicle.h"

using namespace std;

int main() {
    
    AMGGT3 myCar;

    float dt = 0.016f;
    float totalTime = 0.0f;
    int printInterval = 10;

    cout << "==========================================================================================================================================================================\n";
    cout << "                                                    Mercedes-AMG GT3 物理引擎遙測數據主控台 (Console Telemetry)                                             \n";
    cout << "==========================================================================================================================================================================\n";

    cout << setw(6) << "frame" << " | "
        << setw(8) << "status" << " | "
        << setw(8) << "seconds" << " | "
        << setw(5) << "Gear" << " | "
        << setw(7) << "RPM" << " | "
        << setw(14) << "car_vel(KPH)" << " | "
        << setw(15) << "tire_vel(KPH)" << " | "
        << setw(10) << "slip_ratio" << " | "
        << setw(11) << "slip_angle" << " | "
        << setw(13) << "susp FL(m)" << " | "
        << setw(13) << "susp FR(m)" << " | "
        << setw(13) << "susp RL(m)" << " | "
        << setw(13) << "susp RR(m)" << " | "
        << setw(11) << "steer_angle" << " | "
        << setw(14) << "World_Pos(X,Y)" << "\n";
    cout << "--------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";

    int totalFrames = static_cast<int>(39.0f / dt);

    for (int frame = 0; frame <= totalFrames; frame++) {
        float throttle = 0.0f;
        float brakeForce = 0.0f;
        float steerInput = 0.0f;
        string stageStatus = "";

        if (totalTime < 19.0f) {
            throttle = 1.0f; stageStatus = "[ACCEL]";
        }
        else if (totalTime < 21.0f) {
            brakeForce = 15000.0f; stageStatus = "[BRAKE]";
        }
        else if (totalTime < 22.0f) {
            throttle = 0.2f;
            steerInput = 0.025f;
            stageStatus = "[TURN_L]";
        }
        else if (totalTime < 23.0f) {
            throttle = 0.2f;
            steerInput = -0.025f;
            stageStatus = "[TURN_R]";
        }
        else if (totalTime < 29.0f) {
            stageStatus = "[COAST]";
        }
        else if (totalTime < 34.0f) {
            throttle = 1.0f; stageStatus = "[ACCEL]";
        }
        else if (totalTime <= 39.0f) {
            brakeForce = 15000.0f; stageStatus = "[BRAKE]";
        }

        int currentGearIdx = myCar.gearbox.getCurrentGear();
        float currentSpeed = myCar.getKPH();

        if (throttle > 0.1f) {
            if (currentSpeed > 192.0f && currentGearIdx == 6) myCar.gearbox.shiftUp();
            else if (currentSpeed > 153.0f && currentGearIdx == 5) myCar.gearbox.shiftUp();
            else if (currentSpeed > 120.0f && currentGearIdx == 4) myCar.gearbox.shiftUp();
            else if (currentSpeed > 92.0f && currentGearIdx == 3) myCar.gearbox.shiftUp();
            else if (currentSpeed > 68.0f && currentGearIdx == 2) myCar.gearbox.shiftUp();
        }

        if (throttle <= 0.1f) {
            if (currentSpeed < 192.0f && currentGearIdx == 7) myCar.gearbox.shiftDown();
            else if (currentSpeed < 153.0f && currentGearIdx == 6) myCar.gearbox.shiftDown();
            else if (currentSpeed < 120.0f && currentGearIdx == 5) myCar.gearbox.shiftDown();
            else if (currentSpeed < 92.0f && currentGearIdx == 4) myCar.gearbox.shiftDown();
            else if (currentSpeed < 68.0f && currentGearIdx == 3) myCar.gearbox.shiftDown();
        }

        myCar.setSteeringAngle(steerInput);
        myCar.update(throttle, brakeForce, dt);

        totalTime += dt;

        if (frame % printInterval == 0) {
            float tireKPH = myCar.tires[2].tire.getTireVelocity() * 3.6f;
            float refSpeed = max(abs(myCar.getForwardVelocity()), 1.0f);
            float slipRatioVal = (myCar.tires[2].tire.getTireVelocity() - myCar.getForwardVelocity()) / refSpeed;
            float rx_front = 1.3f;
            float wheelVy = myCar.getLateralVelocity() + myCar.getYawRate() * rx_front;
            float slipAngleDeg = (atan2(wheelVy, refSpeed) - steerInput) * (180.0f / PI);

            float suspFL = myCar.getSuspensionLength(0);
            float suspFR = myCar.getSuspensionLength(1);
            float suspRL = myCar.getSuspensionLength(2);
            float suspRR = myCar.getSuspensionLength(3);

            float currentGearRatio = myCar.gearbox.getCurrentRatio();
            float drivenWheelRad = abs(myCar.tires[2].tire.getAngularVel());
            float wheelRPM = drivenWheelRad * (60.0f / TWO_PI);
            float baseEngineRPM = wheelRPM * abs(currentGearRatio) * myCar.gearbox.getFinalDrive();
            float engineRPM = baseEngineRPM;

            if (abs(myCar.getKPH()) < 60.0f && throttle > 0.1f) {
                float launchRPM = myCar.engine.getMaxRPM() * 0.6f;
                float targetLaunchRPM = myCar.engine.getIdleRPM() + throttle * (launchRPM - myCar.engine.getIdleRPM());
                float clutchSlip = 1.0f - (abs(myCar.getKPH()) / 60.0f);
                clutchSlip = clamp(clutchSlip, 0.0f, 1.0f);
                engineRPM = (targetLaunchRPM * clutchSlip) + (baseEngineRPM * (1.0f - clutchSlip));
            }

            engineRPM = max(engineRPM, myCar.engine.getIdleRPM());

            cout << setw(6) << frame << " | "
                << setw(8) << stageStatus << " | "
                << setw(7) << fixed << setprecision(2) << totalTime << "s | "
                << setw(4) << (myCar.gearbox.getCurrentGear() - 1) << "檔 | "
                << setw(7) << setprecision(0) << engineRPM << " | "
                << setw(14) << setprecision(2) << myCar.getKPH() << " | "
                << setw(15) << tireKPH << " | "
                << setw(10) << setprecision(4) << slipRatioVal << " | "
                << setw(11) << setprecision(2) << slipAngleDeg << " | "
                << setw(13) << setprecision(3) << suspFL << " | "
                << setw(13) << setprecision(3) << suspFR << " | "
                << setw(13) << setprecision(3) << suspRL << " | "
                << setw(13) << setprecision(3) << suspRR << " | "
                << setw(11) << setprecision(2) << steerInput << " | "
                << "(" << setprecision(1) << myCar.getWorldX() << "," << myCar.getWorldY() << ")\n";
        }
    }

    return 0;
}