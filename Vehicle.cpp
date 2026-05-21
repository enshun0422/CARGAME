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
        float currentRadius = tires[0].tire.getRadius();

        // 真實由輪速反推的基礎引擎轉速
        float wheelRPM = (this->forwardVelocity / (2.0f * PI * currentRadius)) * 60.0f;
        float baseEngineRPM = std::abs(wheelRPM * currentGearRatio * gearbox.getFinalDrive());

        float engineRPM = baseEngineRPM;

        // 【新增：起步彈射與離合器滑差邏輯】
        // 如果踩油門，且車速低於 60 KPH，允許引擎轉速拉高，模擬離合器半接合
        if (throttle > 0.1f && this->forwardVelocity < 16.6f) { // 16.6 m/s 約為 60 KPH
            // 讓引擎轉速拉升到 4500 轉 (M4 GT3 扭力平原區間)
            float targetLaunchRPM = 4500.0f * throttle;
            if (baseEngineRPM < targetLaunchRPM) {
                engineRPM = targetLaunchRPM;
            }
        }

        // 限制在怠速與紅線之間
        engineRPM = std::clamp(engineRPM, engine.getIdleRPM(), engine.getMaxRPM());

        // 取得引擎扭力並計算傳到驅動軸的總扭力
        float engineTorque = engine.getTorque(engineRPM, throttle);
        float driveTorque = engineTorque * currentGearRatio * gearbox.getFinalDrive() * gearbox.getEfficiency();

        // 假設 RWD (後輪驅動)
        float torquePerDriveTire = driveTorque / 2.0f;

        // 【新增：輪胎最高轉速限制 (Rev Limiter)】
        // 取得引擎最高轉速對應的最高輪速，防止輪胎無限空轉爆炸
        float maxWheelRPM = engine.getMaxRPM() / (currentGearRatio * gearbox.getFinalDrive());
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

            // 【新增：引擎紅線保護】如果輪胎轉速超過引擎極限，切斷扭力輸出
            if (tires[i].tire.getAngularVel() > maxTireAngularVel) {
                currentTorque = 0.0f;
            }

            tires[i].tire.setBreakingForce(brakeForce);

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

        // 【關鍵修復】必須使用 subDt 積分速度
        this->forwardVelocity += accel_X * subDt;
        this->lateralVelocity += accel_Y * subDt;

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


