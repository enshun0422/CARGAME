#include "Vehicle.h"

void Vehicle::setSteeringAngle(float angle) { steeringAngle = angle;}

float Vehicle::getAntiRollBarStiffnessFront() {return antiRollBarStiffnessFront;}
float Vehicle::getAntiRollBarStiffnessRear() {return antiRollBarStiffnessRear;}
float Vehicle::getLateralVelocity() const { return lateralVelocity; }
float Vehicle::getYawRate() const { return yawRate; }
float Vehicle::getTotalMass() const { return totalMass; }
float Vehicle::getForwardVelocity() const { return forwardVelocity; }
float Vehicle::getWorldX() const { return worldX; }
float Vehicle::getWorldY() const { return worldY; }
float Vehicle::getHeadingAngle() const { return headingAngle; }

void Vehicle::update(float throttle, float brakeForce, float dt) {
    throttle = (throttle > 0.02f) ? throttle : 0.0f;
    int subSteps = 10;
    float subDt = dt / subSteps;

    // 將原本的物理邏輯包進子迴圈
    for (int step = 0; step < subSteps; step++) {
        // ==========================================
        // 1. 動力傳遞計算 (Drivetrain)
        // ==========================================
        float currentGearRatio = gearbox.getCurrentRatio();

        // 【修正 1：引擎必須與「驅動輪」的轉速綁定，而不是車速！】
        // RWD 後驅車，我們讀取真實後輪 (tires[2]) 的旋轉角速度 (rad/s)
        float drivenWheelRad = std::abs(tires[2].tire.getAngularVel());
        float wheelRPM = drivenWheelRad * (60.0f / TWO_PI);

        // 【修正 2：齒比加上絕對值，確保打倒檔時引擎轉速依然是正的】
        float baseEngineRPM = wheelRPM * std::abs(currentGearRatio) * gearbox.getFinalDrive();

        float engineRPM = baseEngineRPM;

        // 起步彈射與離合器滑差邏輯 (保持原樣，這段沒問題)
        if (throttle > 0.1f && std::abs(this->forwardVelocity) < 16.6f) { // 16.6 m/s 約為 60 KPH
            float targetLaunchRPM = 4500.0f * throttle;
            if (baseEngineRPM < targetLaunchRPM) {
                engineRPM = targetLaunchRPM;
            }
        }

        // 【修正 3：解除紅線封印！】
        // 只限制最低怠速防熄火，絕對不能用 clamp 限制最高轉速。
        // 必須讓 engineRPM 有機會突破 7500，才能觸發 Engine 裡的負扭力 (斷油保護)！
        engineRPM = std::max(engineRPM, engine.getIdleRPM());

        // 【新增：TCS 循跡防滑系統介入】
        float actualThrottle = throttle;
        if (TCSActive) {
            // 如果監測到後輪 (驅動輪) 的滑移率飆升超過 10% (0.1f)
            if (tires[2].getSlipRatio() > 0.1f || tires[3].getSlipRatio() > 0.1f) {
                // 電腦強制切斷油門，只保留 5% 的動力讓輪胎恢復側向抓地力
                actualThrottle = 0.05f;
            }
        }

        // 使用被 TCS 修正過的油門來計算扭力
        float engineTorque = engine.getTorque(engineRPM, actualThrottle);
        float driveTorque = engineTorque * currentGearRatio * gearbox.getFinalDrive() * gearbox.getEfficiency();


        // 假設 RWD (後輪驅動)
        float torquePerDriveTire = driveTorque / 2.0f;

        // 【新增：輪胎最高轉速限制 (Rev Limiter)】
        // 取得引擎最高轉速對應的最高輪速，防止輪胎無限空轉爆炸
        float maxWheelRPM = 100000.0f;
        if (std::abs(currentGearRatio) > 0.01f) {
            maxWheelRPM = engine.getMaxRPM() / std::abs(currentGearRatio * gearbox.getFinalDrive());
        }
        float maxTireAngularVel = maxWheelRPM * (2.0f * PI / 60.0f); // 轉為 rad/s

        

        // ==========================================
        // 2. 空氣動力與防傾桿計算
        // ==========================================
        float currentDrag = aero.calculateDragForce(this->forwardVelocity);
        float currentFrontDF = aero.calculateFrontDownforce(this->forwardVelocity);
        float currentRearDF = aero.calculateRearDownforce(this->forwardVelocity);

        // 模擬的 Raycast 壓縮量，這裡用你標頭檔宣告的預設變數
        float compression[4] = { 0.05f, 0.05f, 0.05f, 0.05f };
        float frontAntiRollForce = (compression[0] - compression[1]) * antiRollBarStiffnessFront;
        float rearAntiRollForce = (compression[2] - compression[3]) * antiRollBarStiffnessRear;

        // 宣告車體總受力變數 (修復未定義錯誤)
        float total_Fx_car = 0.0f;
        float total_Fy_car = 0.0f;
        float total_YawTorque = 0.0f;
        // ==========================================
        // 3. 四輪獨立運算 (Suspension & Tire)
        // ==========================================
        for (int i = 0; i < 4; i++) {

            // -- A. 計算正向力 Fz --
            float staticWeight = (this->totalMass * 9.81f) / 4.0f;
            float aeroDownforce = (i < 2) ? currentFrontDF / 2.0f : currentRearDF / 2.0f;

            float arbForce = 0.0f;
            if (i == 0) arbForce = frontAntiRollForce;       // 左前：如果自己壓比較深，ARB 會把你往上頂 (減載)
            if (i == 1) arbForce = -frontAntiRollForce;      // 右前：如果左前壓比較深，ARB 會把你往下壓 (加載)
            if (i == 2) arbForce = rearAntiRollForce;        // 左後
            if (i == 3) arbForce = -rearAntiRollForce;       // 右後

            // 計算這顆輪胎最終的動態 Fz (為了簡化，這裡暫時將靜態重量和 ARB 力結合)
            float final_Fz = staticWeight + aeroDownforce + arbForce;
            // 確保輪胎不會有負的載重 (輪胎離地)
            final_Fz = std::max(0.0f, final_Fz);
            tires[i].setVerticalLoad(final_Fz);

            // --- B. 輪胎旋轉積分與打滑狀態 ---
            float currentTorque = (i >= 2) ? torquePerDriveTire : 0.0f;

            // 引擎紅線保護
            if (std::abs(tires[i].tire.getAngularVel()) > maxTireAngularVel) {
                currentTorque = 0.0f;
            }

            // 【修正：重構 ABS 防鎖死邏輯】
            float appliedBrakeForce = brakeForce;

            if (ABSActive && brakeForce > 0.0f) {
                // optimalBrakeSlip 之前設定為 -0.15
                if (tires[i].getSlipRatio() < optimalBrakeSlip) {
                    // 當滑移率低於極限，必須「徹底放開」煞車，讓輪胎靠地面摩擦力重新轉起來！
                    // 不要只乘以 0.5，直接設為 0 (或極小值) 才能在下一個 subStep 瞬間解除鎖死
                    appliedBrakeForce = 0.0f;
                }
            }
            // 必須傳遞修改後的 appliedBrakeForce，而不是原本的 brakeForce
            tires[i].tire.setBreakingForce(appliedBrakeForce);

            // 阻力設定
            float currentFriction = tires[i].getLongitudinalForce();
            tires[i].tire.setFriction(currentFriction);

            // 【關鍵修復】必須使用 subDt 積分轉速
            tires[i].tire.integrateRotation(currentTorque, subDt);

            // --- C. 計算 Pacejka 與側向力 ---
            // 只有前輪有轉向角
            float currentSteer = (i < 2) ? steeringAngle : 0.0f;

            // 更新滑移率
            tires[i].calculslipRatio(this->forwardVelocity);

            // 【關鍵修復】更新滑移角 (前一版本完全遺漏)
            tires[i].calculateSlipAngle(this->forwardVelocity, this->lateralVelocity, this->yawRate, tirePos_X[i], tirePos_Y[i], currentSteer);

            // 假設 Camber 為 0 (若後續接上 Suspension 可再修改)
            tires[i].setCamberAngle(0.0f);

            // 執行魔術公式與摩擦橢圓截斷
            tires[i].updateSlipState();

            // --- D. 座標投影與總力加總 ---
            float Fx_tire = tires[i].getLongitudinalForce();
            float Fy_tire = tires[i].getLateralForce();

            float cosSteer = std::cos(currentSteer);
            float sinSteer = std::sin(currentSteer);

            // 將輪胎受力投影到車體座標系 (Steering Transformation)
            float Fx_car = Fx_tire * cosSteer - Fy_tire * sinSteer;
            float Fy_car = Fx_tire * sinSteer + Fy_tire * cosSteer;

            total_Fx_car += Fx_car;
            total_Fy_car += Fy_car;

            // 計算轉向力矩 (力臂外積)
            float torque_Z = (tirePos_X[i] * Fy_car) - (tirePos_Y[i] * Fx_car);
            total_YawTorque += torque_Z;
        }

        // ==========================================
        // 4. 車身剛體更新 (Rigid Body Integration)
        // ==========================================

        // 扣除空氣阻力
        total_Fx_car -= currentDrag;

        // 計算加速度
        float accel_X = total_Fx_car / this->totalMass;
        float accel_Y = total_Fy_car / this->totalMass;

        // 加入旋轉座標系的離心力/科氏力補償
        // 公式：v_dot = a_y - (u * r)
        // 公式：u_dot = a_x + (v * r)
        float delta_Vx = accel_X + (this->lateralVelocity * this->yawRate);
        float delta_Vy = accel_Y - (this->forwardVelocity * this->yawRate);

        this->forwardVelocity += delta_Vx * subDt;
        this->lateralVelocity += delta_Vy * subDt;

        // 數值穩定保護
        if (this->forwardVelocity < 0.01f && driveTorque == 0.0f && accel_X <= 0.0f) {
            this->forwardVelocity = 0.0f;
        }

        // 更新角速度
        float inertia_Z = 2000.0f;
        float angularAccel_Z = total_YawTorque / inertia_Z;
        this->yawRate += angularAccel_Z * subDt;

        // ==========================================
        // 5. 座標系統轉換與世界狀態更新
        // ==========================================
        this->headingAngle += this->yawRate * subDt;

        if (this->headingAngle > PI)  this->headingAngle -= TWO_PI;
        if (this->headingAngle < -PI) this->headingAngle += TWO_PI;

        float cosHeading = std::cos(this->headingAngle);
        float sinHeading = std::sin(this->headingAngle);

        float worldVelocity_X = (this->forwardVelocity * cosHeading) - (this->lateralVelocity * sinHeading);
        float worldVelocity_Y = (this->forwardVelocity * sinHeading) + (this->lateralVelocity * cosHeading);

        this->worldX += worldVelocity_X * subDt;
        this->worldY += worldVelocity_Y * subDt;
    }
    
}

float Vehicle::getKPH() {
    return this->forwardVelocity * 3.6f; // m/s 轉 km/h
}


