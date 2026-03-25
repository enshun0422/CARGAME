#include "Wheel.h"
#include "Vehicle.h"
#include <iostream>
#include <cmath>

void Wheel::update(float driveTorque, float dt) {
	float frictionTorque = Fx * radius; // 摩擦力轉換為扭矩
    
	float netTorque = driveTorque - frictionTorque; // 扣除摩擦力後的淨扭矩

	float angularAccel = netTorque / inertia;//∑Torque = I * alpha -> alpha = Torque / I
    
    angularVel += angularAccel * dt;

    if (std::abs(angularVel) < 0.01f) {
        angularVel = 0.0f; // 防止反轉
	}
}

float Wheel::getTireVelocity() {
    return angularVel * radius;
}

float Wheel::calculslipRatio(float forwardVelocity) {
    float wheelV = getTireVelocity();
    // 只要確保分母不為 0 即可，使用物理上較穩定的參考速度
    float referenceSpeed = std::max(std::abs(forwardVelocity), 1.0f);
    slipRatio = (wheelV - forwardVelocity) / referenceSpeed;
    return slipRatio;
}

float Wheel::updateSlipState() {
	Fx = slipRatio * 1000.0f; 
	return Fx;
    /*
    *簡化的摩擦力模型，實際應該根據輪胎特性曲線計算。此處的
    *係指(Slip Stiffness)。在後續優化中，將引入 Pacejka Magic Formula，
    *透過非線性函數來模擬輪胎在極限狀態下的失速與打滑特性。
    */ 
    
}
/*備用版本，使用 Pacejka Magic Formula 計算縱向力 Fx，提供更真實的輪胎行為模擬。
void Wheel::updateSlipState() {
    float Fz = 3000.0f; // 假設載重
    this->Fx = calculatePacejkaFx(this->slipRatio, Fz);
}
*/

/**備用
 * 
 * @brief 計算縱向力 Fx (Magic Formula 簡化版)
 * @param s 滑移率 (slipRatio)，通常在 -1.0 到 1.0 之間
 * @param Fz 垂直載重 (垂直壓在輪胎上的力)，這會決定抓地力上限
 */
float Wheel::calculatePacejkaFx(float s, float Fz) {
    // --- 這裡的四個參數決定了輪胎的性格 ---
    float B = 10.0f;  // Stiffness Factor (剛性因子)：數值越大，斜率越陡（越靈敏）
    float C = 1.5f;   // Shape Factor (形狀因子)：通常固定在 1.5 左右，決定曲線形狀
    float D = 1.0f;   // Peak Factor (峰值因子)：代表摩擦係數 (mu)，1.0 是一般輪胎
    float E = 0.97f;  // Curvature Factor (曲率因子)：影響過頂點後的下降坡度

    // 計算公式：Fx = Fz * D * sin(C * atan(B*s - E * (B*s - atan(B*s))))
    float Bs = B * s;
    float force = Fz * D * sin(C * atan(Bs - E * (Bs - atan(Bs))));

    return force;
}
