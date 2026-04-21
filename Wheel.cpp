#include "Wheel.h"


void Wheel::setFriction(float friction) {
	fabsf(friction) < 0.01f ? this->roadFriction = 0.0f : this->roadFriction = friction; // 避免過小的摩擦力導致反轉

}

void Wheel::setBreakingForce(float breakingForce) {
	fabsf(breakingForce) < 0.01f ? this->breakingForce = 0.0f : this->breakingForce = breakingForce; // 避免過小的制動力導致反轉
}


float Wheel::calculateAngularAcceleration(float driveTorque) {
	float frictionTorque = roadFriction * radius; // 摩擦力轉換為扭矩
	float breakingTorque = breakingForce * radius; // 制動力轉換為扭矩
	
	float netTorque = driveTorque - frictionTorque - breakingTorque; // 扣除摩擦力和制動力後的淨扭矩
	float angularAccel = netTorque / inertia;//∑Torque = I * alpha -> alpha = Torque / I
	return angularAccel;
}

void Wheel::integrateRotation(float driveTorque, float dt) {
    // 1. 先算出原本的角加速度與「預計」的新角速度
    float accel = calculateAngularAcceleration(driveTorque);
    float newAngularVel = angularVel + accel * dt;

    // 2. 數值震盪保護機制 (針對加速狀態)
    // 如果我們正在踩油門 (driveTorque > 0)，且原本輪子是往前轉的 (angularVel >= 0)
    // 但算出來的新輪速卻變成負的 (newAngularVel < 0)
    // 代表「地面摩擦力太大了，過度修正了！」
    if (driveTorque > 0.0f && angularVel >= 0.0f && newAngularVel < 0.0f) {
        // 強制把輪速限制在 0 (或是極微小的正數)，不准它倒轉
        newAngularVel = 0.001f;
    }

    // 3. 數值震盪保護機制 (針對煞車狀態)
    // 同理，如果沒踩油門但在重煞車，輪速也不該從正的瞬間變成負的
    if (driveTorque == 0.0f && angularVel >= 0.0f && newAngularVel < 0.0f) {
        newAngularVel = 0.0f; // 煞車最多只能煞到停，不能倒車
    }

    // 更新真正受保護的角速度
    angularVel = newAngularVel;

    // 更新角位置與切線速度
    rotationAngle += angularVel * dt;
    rotationAngle = fmodf(rotationAngle, 2.0f * 3.141592);
    tangentialVelocity = angularVel * radius;
}


float Wheel::getTireVelocity() const {
    return angularVel * radius;
}




