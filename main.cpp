#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <string>
#include "Porsche911GT3R.h"
#include "BMWM4GT3.h"
#include "AMGGT3.h"
#include "Vehicle.h"

using namespace std;

int main() {

    BMWM4GT3 bmw;
    Porsche911GT3R porsche;
    AMGGT3 amg;
    Vehicle* carArray[3] = { &bmw, &porsche, &amg };

	int choise = 0;
	cout << "請選擇要模擬的車輛:\n";
    cout << "1. BMW M4 GT3\n2. Porsche 911 GT3 R\n3. Mercedes-AMG GT3\n請輸入'1', '2' 或 '3': ";
    cin >> choise;
	Vehicle* myCar = nullptr;
    if (choise < 1 || choise > 3) {
        cout << "無效的選擇" << endl;
        return 1;
    } else if (choise == 1) {
        myCar = carArray[0];
    } else if (choise == 2) {
        myCar = carArray[1];
    } else if (choise == 3) {
        myCar = carArray[2];
    }

    float dt = 0.016f;
    float totalTime = 0.0f;
    int printInterval = 10;

    cout << "==========================================================================================================================================================================\n";
    cout << "                                                    ";
    cout << myCar->getName();
    cout << "物理引擎遙測數據主控台(Console Telemetry)                                             \n";
    cout << "==========================================================================================================================================================================\n";

    cout << setw(6) << "frame" << " | "
        << setw(8) << "status" << " | "
        << setw(8) << "seconds" << " | "
        << setw(5) << "Gear" << " | "
        << setw(7) << "RPM" << " | "
        << setw(14) << "car_speed(KPH)" << " | "
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

        int currentGearIdx = myCar->gearbox.getCurrentGear();
        float currentEngineRPM = myCar->estimateEngineRPM(throttle);

        if (currentGearIdx >= 2) {
            if (throttle > 0.1f && currentEngineRPM >= myCar->getShiftUpRPM() && currentGearIdx < 7) {
                myCar->gearbox.shiftUp();
            }
            else if (currentEngineRPM <= myCar->getShiftDownRPM() && currentGearIdx > 2) {
                myCar->gearbox.shiftDown();
            }
        }

        myCar->setSteeringAngle(steerInput);
        myCar->update(throttle, brakeForce, dt);

        totalTime += dt;

        if (frame % printInterval == 0) {
            float tireKPH = myCar->tires[2].tire.getTireVelocity() * 3.6f;
            float refSpeed = max(abs(myCar->getForwardVelocity()), 1.0f);
            float slipRatioVal = (myCar->tires[2].tire.getTireVelocity() - myCar->getForwardVelocity()) / refSpeed;
            float rx_front = 1.3f;
            float wheelVy = myCar->getLateralVelocity() + myCar->getYawRate() * rx_front;
            float slipAngleDeg = (atan2(wheelVy, refSpeed) - steerInput) * (180.0f / PI);

            float suspFL = myCar->getSuspensionLength(0);
            float suspFR = myCar->getSuspensionLength(1);
            float suspRL = myCar->getSuspensionLength(2);
            float suspRR = myCar->getSuspensionLength(3);

            float engineRPM = myCar->estimateEngineRPM(throttle);

            cout << setw(6) << frame << " | "
                << setw(8) << stageStatus << " | "
                << setw(7) << fixed << setprecision(2) << totalTime << "s | "
                << setw(4) << (myCar->gearbox.getCurrentGear() - 1) << "檔 | "
                << setw(7) << setprecision(0) << engineRPM << " | "
                << setw(14) << setprecision(2) << myCar->getSpeedKPH() << " | "
                << setw(15) << tireKPH << " | "
                << setw(10) << setprecision(4) << slipRatioVal << " | "
                << setw(11) << setprecision(2) << slipAngleDeg << " | "
                << setw(13) << setprecision(3) << suspFL << " | "
                << setw(13) << setprecision(3) << suspFR << " | "
                << setw(13) << setprecision(3) << suspRL << " | "
                << setw(13) << setprecision(3) << suspRR << " | "
                << setw(11) << setprecision(2) << steerInput << " | "
                << "(" << setprecision(1) << myCar->getWorldX() << "," << myCar->getWorldY() << ")\n";
        }
    }

    return 0;
}
