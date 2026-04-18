#include "TireDynamics.h"


void TireDynamics::setLongitudinalForce(float fx) {
	Fx = fx;
}

void TireDynamics::setLateralForce(float fy) {
	Fy = fy;
}

void TireDynamics::setVerticalLoad(float load) {
	Fz = load;
}

float TireDynamics::calculatePacejkaFx(float s, float Fz, float B, float C, float D, float E) {
	
	// 計算公式：Fx = Fz * D * sin(C * atan(B*s - E * (B*s - atan(B*s))))
	float Bs = B * s;
	float force = Fz * D * sin(C * atan(Bs - E * (Bs - atan(Bs))));
	return force;
}

float TireDynamics::calculslipRatio(float forwardVelocity) {
	float wheelV = wheel.getTireVelocity();
	// 只要確保分母不為 0 即可，使用物理上較穩定的參考速度
	float referenceSpeed = std::max(std::abs(forwardVelocity), 1.0f);
	slipRatio = (wheelV - forwardVelocity) / referenceSpeed;
	return slipRatio;
}



void TireDynamics::updateTireForces() {
	// 這裡可以根據 slipRatio 和 slipAngle 更新 Fx 和 Fy
	// 目前先簡化為 Fx = slipRatio * Fz，Fy = slipAngle * Fz
	Fx = slipRatio * Fz;
	Fy = slipAngle * Fz;
}

void TireDynamics::updateSlipState() {
	// 使用物件中的垂直載重（若未設置則使用預設值）
	float Fz_val = (this->Fz != 0.0f) ? this->Fz : 2000.0f; // 假設載重

	this->Fx = calculatePacejkaFx(this->slipRatio, Fz_val, Default_B, Default_C, Default_D, Default_E);
}

float TireDynamics::getLongitudinalForce() const {
	return Fx;
}
