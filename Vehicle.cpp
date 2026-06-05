#include "Vehicle.h"

void Vehicle::setSteeringAngle(float angle) { steeringAngle = angle;}
void Vehicle::setAntiRollBarStiffnessFront(float stiffness) { antiRollBarStiffnessFront = stiffness; }
void Vehicle::setAntiRollBarStiffnessRear(float stiffness) { antiRollBarStiffnessRear = stiffness; }
void Vehicle::setlastSuspensionLoad(int index, float length) {
    if(index >= 0 && index < 4)
		lastSuspensionLength[index] = length;
}
void Vehicle::setCgHeight(float cgheight) { this->cgHeight = cgheight; }
void Vehicle::setTotalMass(float mass) { this->totalMass = mass; }
void Vehicle::setAxleDistances(float front, float rear) {
	this->cgToFrontAxle = front;
    this->cgToRearAxle = -std::abs(rear);

    tirePos_X[0] = cgToFrontAxle;
    tirePos_X[1] = cgToFrontAxle;
    tirePos_X[2] = cgToRearAxle;
    tirePos_X[3] = cgToRearAxle;
}
void Vehicle::setHalfTrackWidth(float width) {
	halfTrackWidth = width;

	tirePos_Y[0] = -halfTrackWidth;
	tirePos_Y[1] = halfTrackWidth;
	tirePos_Y[2] = -halfTrackWidth;
	tirePos_Y[3] = halfTrackWidth;
}

float Vehicle::getAntiRollBarStiffnessFront() {return antiRollBarStiffnessFront;}
float Vehicle::getAntiRollBarStiffnessRear() {return antiRollBarStiffnessRear;}
float Vehicle::getLateralVelocity() const { return lateralVelocity; }
float Vehicle::getYawRate() const { return yawRate; }
float Vehicle::getTotalMass() const { return totalMass; }
float Vehicle::getForwardVelocity() const { return forwardVelocity; }
float Vehicle::getWorldX() const { return worldX; }
float Vehicle::getWorldY() const { return worldY; }
float Vehicle::getHeadingAngle() const { return headingAngle; }
float Vehicle::getSuspensionLoad(int index) const {
    if (index < 0 || index >= 4) return 0.0f;
    return lastSuspensionLoad[index];
}
float Vehicle::getSuspensionLength(int index) const {
    if (index < 0 || index >= 4) return 0.0f;
    return lastSuspensionLength[index];
}

void Vehicle::update(float throttle, float brakeForce, float dt) {
    throttle = (throttle > 0.02f) ? throttle : 0.0f;
    int subSteps = 10;
    float subDt = dt / subSteps;

    // 1. 進入迴圈時，第 2 步的懸吊可以合法讀取到「上一個子步」留下來的加速度。
    // 2. 到了第 4 步更新這兩個值後，它們會存活並帶入下一次的 step 迴圈中。
    float accel_X = 0.0f;
    float accel_Y = 0.0f;

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
        if (throttle > 0.8f && std::abs(this->forwardVelocity) < 5.0f) { // 5.0 m/s 約為 18 KPH
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
                actualThrottle = std::min(throttle, 0.05f);
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
        // 計算加速度 (Local Frame)

        float currentDrag = aero.calculateDragForce(this->forwardVelocity);
        float currentFrontDF = aero.calculateFrontDownforce(this->forwardVelocity);
        float currentRearDF = aero.calculateRearDownforce(this->forwardVelocity);

        // 【新增】利用上一幀的加速度，計算重心轉移導致的車身姿態
        // 為了數值穩定，我們假設懸吊有一個極短的反應時間 (這裡簡化處理)
        float pitchAccel = accel_X * cgHeight; // 煞車時 accel_X 為負，這裡的 Pitch 設定為簡化比例
        float rollAccel = accel_Y * cgHeight;  // 轉彎時的側向力矩

        // 簡化的姿態更新 (帶有阻尼衰減，避免無窮震盪)
        this->currentPitch = this->currentPitch * 0.8f - (pitchAccel * 0.0001f);
        this->currentRoll = this->currentRoll * 0.8f + (rollAccel * 0.0001f);

        // 根據車身姿態 (Pitch/Roll) 計算四個避震器的「真實壓縮量」
        // suspension[0]: 左前, [1]: 右前, [2]: 左後, [3]: 右後
        float baseLength = suspensions[0].getRestLength(); // 預設靜態懸吊長度
        float currentLengths[4];

        // 幾何投影：透過車身傾斜角度，算出四個角落的懸吊被壓了多少
        // Vehicle.cpp 的第 2 步尾端：
        // 幾何投影算出長度後，強制限制在 0.05m (5公分) 到 0.35m (35公分) 之間，絕對不准變成負數！
        currentLengths[0] = std::clamp(baseLength - (this->currentPitch * cgToFrontAxle) + (this->currentRoll * halfTrackWidth), 0.05f, 0.35f);
        currentLengths[1] = std::clamp(baseLength - (this->currentPitch * cgToFrontAxle) - (this->currentRoll * halfTrackWidth), 0.05f, 0.35f);
        currentLengths[2] = std::clamp(baseLength + (this->currentPitch * std::abs(cgToRearAxle)) + (this->currentRoll * halfTrackWidth), 0.05f, 0.35f);
        currentLengths[3] = std::clamp(baseLength + (this->currentPitch * std::abs(cgToRearAxle)) - (this->currentRoll * halfTrackWidth), 0.05f, 0.35f);

        // 計算真實的防傾桿力量 (根據左右避震器的長度差)
        float frontAntiRollForce = (currentLengths[0] - currentLengths[1]) * antiRollBarStiffnessFront;
        float rearAntiRollForce = (currentLengths[2] - currentLengths[3]) * antiRollBarStiffnessRear;

        // 宣告車體總受力變數 (修復未定義錯誤)
        float total_Fx_car = 0.0f;
        float total_Fy_car = 0.0f;
        float total_YawTorque = 0.0f;
        // ==========================================
        // 3. 四輪獨立運算 (Suspension & Tire)
        // ==========================================
        for (int i = 0; i < 4; i++) {

            // -- A. 計算正向力 Fz --
            float suspensionForce = suspensions[i].calculateForce(currentLengths[i], dt);
            float aeroDownforce = (i < 2) ? currentFrontDF / 2.0f : currentRearDF / 2.0f;

            float arbForce = 0.0f;
            if (i == 0) arbForce = frontAntiRollForce;       // 左前：如果自己壓比較深，ARB 會把你往上頂 (減載)
            if (i == 1) arbForce = -frontAntiRollForce;      // 右前：如果左前壓比較深，ARB 會把你往下壓 (加載)
            if (i == 2) arbForce = rearAntiRollForce;        // 左後
            if (i == 3) arbForce = -rearAntiRollForce;       // 右後

            // 計算這顆輪胎最終的動態 Fz (為了簡化，這裡暫時將靜態重量和 ARB 力結合)
            float final_Fz = std::max(0.0f, suspensionForce + aeroDownforce + arbForce);
            lastSuspensionLength[i] = currentLengths[i];
            lastSuspensionLoad[i] = final_Fz;
            tires[i].setVerticalLoad(final_Fz);

            // -- B. 觀察現狀：計算 Slip Ratio 與 Slip Angle --
            float currentSteer = (i < 2) ? steeringAngle : 0.0f;

            // 根據當前的車速與輪速，計算滑移狀態
            tires[i].calculslipRatio(this->forwardVelocity);
            tires[i].calculateSlipAngle(this->forwardVelocity, this->lateralVelocity, this->yawRate, tirePos_X[i], tirePos_Y[i], currentSteer);
            tires[i].setCamberAngle(0.0f);

            // -- C. 產生作用力：Pacejka 魔術公式 --
            // 根據剛剛算出的 Slip，產生這一幀真正的縱向力與側向力
            tires[i].updateSlipState();

            // -- D. 準備積分：設定輪胎的受力邊界條件 --
            // 1. 設定輪胎遭遇的地面摩擦阻力
            float currentFriction = tires[i].getLongitudinalForce();
            tires[i].tire.setFriction(currentFriction);

            // 2. 設定引擎傳來的驅動扭矩 (附帶紅線保護)
            float currentTorque = (i >= 2) ? torquePerDriveTire : 0.0f;
            if (std::abs(tires[i].tire.getAngularVel()) > maxTireAngularVel) {
                currentTorque = 0.0f;
            }

            // 3. 設定煞車卡鉗傳來的制動力 (附帶 ABS 邏輯)
            float appliedBrakeForce = brakeForce;
            if (ABSActive && brakeForce > 0.0f) {
                if (tires[i].getSlipRatio() < optimalBrakeSlip) {
                    appliedBrakeForce = 0.0f; // 釋放煞車避免鎖死
                }
                if (std::abs(forwardVelocity) < 4.0f) {
                    appliedBrakeForce *= 0.4f;
                }
            }
            tires[i].tire.setBreakingForce(appliedBrakeForce);

            // -- E. 推進時間：積分輪胎旋轉速度 --
            // 力都算完了，最後才來積分速度！
            tires[i].tire.integrateRotation(currentTorque, subDt);

            // -- F. 座標投影與總力加總 --
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
        accel_X = total_Fx_car / this->totalMass;
        accel_Y = total_Fy_car / this->totalMass;

        // 加入旋轉座標系的離心力/科氏力補償
        // 公式：v_dot = a_y - (u * r)
        // 公式：u_dot = a_x + (v * r)
        float delta_Vx = accel_X + (this->lateralVelocity * this->yawRate);
        float delta_Vy = accel_Y - (this->forwardVelocity * this->yawRate);

        this->forwardVelocity += delta_Vx * subDt;
        this->lateralVelocity += delta_Vy * subDt;

        // 數值穩定保護
        if (brakeForce > 0.0f && std::abs(this->forwardVelocity) < 0.2f && std::abs(tires[2].tire.getAngularVel()) < 0.5f) {
            this->forwardVelocity = 0.0f;
        }
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


