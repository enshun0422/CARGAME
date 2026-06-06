#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include<string>
#include "Engine.h"
#include "Gearbox.h"
#include "Suspension.h"
#include "AeroDynamics.h"
#include "Tire.h"
#include "TireDynamics.h"

constexpr float PI = 3.1415926535f;
constexpr float TWO_PI = 6.2831853071f;

class Vehicle {
private:
    float yawRate = 0.0f; // 車體旋轉角速度 (rad/s)
	float lateralVelocity = 0.0f; // 車體側向滑動速度 (m/s)
	float steeringAngle = 0.0f; // 前輪轉向角 (rad)

    float totalMass = 1300.0f; // 噸
    float cgToFrontAxle = 1.458f;  // 重心到前軸距離 (rx)
    float cgToRearAxle = -1.459f;  // 重心到後軸距離 (rx)
    float halfTrackWidth = 0.96f; // 左右輪距的一半 (ry)

    float tirePos_X[4] = { cgToFrontAxle, cgToFrontAxle, cgToRearAxle, cgToRearAxle };
    float tirePos_Y[4] = { -halfTrackWidth, halfTrackWidth, -halfTrackWidth, halfTrackWidth };

    float antiRollBarStiffnessFront = 80000.0f; // 前防傾桿
    float antiRollBarStiffnessRear = 60000.0f;  // 後防傾桿

	float currentSuspensionLength = 0.3f; // 當前懸吊長度 (m)，這裡假設四個輪子一樣

    float currentPitch = 0.0f; // 俯仰角 (煞車為正，加速為負)
    float currentRoll = 0.0f;  // 側傾角 (左轉為正，右轉為負)
    float cgHeight = 0.45f;    // 假設重心高度 45 公分

    float forwardVelocity = 0.0f; // 前進速度 (m/s)

    float lastSuspensionLoad[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float lastSuspensionLength[4] = { 0.2f, 0.2f, 0.2f, 0.2f };

	float RPM = 0.0f; // 引擎轉速 (RPM)
    float shiftUpRPM = 7100.0f;
    float shiftDownRPM = 3500.0f;

	// 電子輔助系統狀態
	bool ABSActive = true; // 防鎖死煞車系統 (ABS) 是否啟動
	bool TCSActive = true; // 循跡控制系統 (TCS) 是否啟動
    float optimalBrakeSlip = -0.15f;

    // --- 世界狀態變數 ---
    float worldX = 0.0f;       // 世界座標 X (公尺)
    float worldY = 0.0f;       // 世界座標 Y (公尺)
    float headingAngle = 0.0f; // 世界朝向角 (弧度，0 代表面向 +X 軸)
public:
	Engine engine;
	Gearbox gearbox;
    TireDynamics tires[4];
	AeroDynamics aero;
    // 防傾桿剛性 (Anti-Roll Bar Stiffness)，單位通常是 N/m 或是 Nm/rad
    // 這裡我們簡化為：當左右懸吊行程相差 1 公尺時，轉移多少牛頓的力

    Suspension suspensions[4] = {
    Suspension(0.2f, 120000.0f, 8000.0f), // 左前
    Suspension(0.2f, 120000.0f, 8000.0f), // 右前
    Suspension(0.2f, 140000.0f, 9000.0f), // 左後 (為了支撐加速重心轉移，後彈簧常稍硬)
    Suspension(0.2f, 140000.0f, 9000.0f)  // 右後
    };

	virtual ~Vehicle() = default;

    void setSteeringAngle(float angle);
    void setAntiRollBarStiffnessFront(float stiffness);
    void setAntiRollBarStiffnessRear(float stiffness);
	void setlastSuspensionLoad(int index, float load);
    void setCgHeight(float height);
	void setTotalMass(float mass);
	void setAxleDistances(float front, float rear);
	void setHalfTrackWidth(float width);
    void setShiftRPM(float upRPM, float downRPM);

    float getAntiRollBarStiffnessFront();
	float getAntiRollBarStiffnessRear();
    float getLateralVelocity() const;
    float getYawRate() const;
    float getTotalMass() const;
    float getForwardVelocity() const;
    float getWorldX() const;
    float getWorldY() const;
	float getHeadingAngle() const;
    float getSuspensionLoad(int index) const;
    float getSuspensionLength(int index) const;
    float getShiftUpRPM() const;
    float getShiftDownRPM() const;
    float estimateEngineRPM(float throttle) const;
    float getSpeedKPH() const;
    
	virtual void update(float throttle, float brakeForce, float dt);
    float getKPH();
    virtual std::string getName();
	
};
