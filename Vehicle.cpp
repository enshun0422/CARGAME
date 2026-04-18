#include "Vehicle.h"



void Vehicle::update(float driveTorque, float brakeForce, float dt) {

    int subSteps = 10;
    float subDt = dt / subSteps;

    // 將原本的物理邏輯包進子迴圈
    for (int step = 0; step < subSteps; step++) {

        float totalFx = 0.0f; // 用來收集四顆輪胎產生的總力 (前進推力或煞車阻力)

        // 假設這是一台後輪驅動車 (RWD)：引擎動力只分給左後(2)與右後(3)輪
        float torquePerDriveWheel = driveTorque / 2.0f;

        // --- 階段一到階段三：個別更新四顆輪胎 ---
        for (int i = 0; i < 4; i++) {

            // 1. 分配駕駛輸入 (煞車四輪都有，油門只有後輪有)
            float currentTorque = (i >= 2) ? torquePerDriveWheel : 0.0f;
            tires[i].wheel.setBreakingForce(brakeForce);

            // 2. 【閉環回饋關鍵】讀取這顆輪胎上一幀算出的摩擦力，傳給 Wheel 當作阻力
            float currentFriction = tires[i].getLongitudinalForce();
            tires[i].wheel.setFriction(currentFriction);

            // 3. 轉動積分：計算角加速度並更新輪速 (呼叫你寫的 Wheel::integrateRotation)
            tires[i].wheel.integrateRotation(currentTorque, dt);

            // 4. 計算滑移率：比對輪速與目前的「車速」(呼叫你寫的 TireDynamics::calculslipRatio)
            tires[i].calculslipRatio(this->forwardVelocity);

            // 5. Pacejka 魔術公式：算出這顆輪胎此時此刻產生的真實抓地力 (Fx)
            tires[i].updateSlipState();

            // 6. 將這顆輪胎新算出來的 Fx 加入總和中
            totalFx += tires[i].getLongitudinalForce();
        }

        // --- 階段四：車身整體物理更新 ---

        // 根據牛頓第二運動定律：a = F / m
        float acceleration = totalFx / this->totalMass;

        // 積分更新車速：v = v0 + at
        this->forwardVelocity += acceleration * dt;

        // 【數值穩定保護】如果沒踩油門，且車速已經趨近於 0，就強制歸零，避免被 Pacejka 算出的微小反向力推著倒退嚕
        if (this->forwardVelocity < 0.01f && driveTorque == 0.0f) {
            this->forwardVelocity = 0.0f;
        }

        /* 積分更新位移：s = s0 + vt (選用，之後想算車子跑多遠)
        this->position += this->forwardVelocity * dt;
        */

    }
}

float Vehicle::getKPH() {
    return this->forwardVelocity * 3.6f; // m/s 轉 km/h
}


