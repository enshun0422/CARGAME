#include "TireDynamics.h"


void TireDynamics::setLongitudinalForce(float fx) {Fx = fx;}
void TireDynamics::setLateralForce(float fy) {Fy = fy;}
void TireDynamics::setVerticalLoad(float load) {Fz = load;}
void TireDynamics::setCamberAngle(float camber) { camberAngle = camber; }
float TireDynamics::getLongitudinalForce() const {return Fx;}
float TireDynamics::getLateralForce() const {return Fy;}
float TireDynamics::getAngularVel(Tire tire) const {
	float angularVel = tire.getAngularVel();
    return angularVel;
}

// ---計算狀態---
float TireDynamics::calculslipRatio(float forwardVelocity) {
	float tireV = tire.getTireVelocity();
	// 只要確保分母不為 0 即可，使用物理上較穩定的參考速度
	float referenceSpeed = std::max(std::abs(forwardVelocity), 1.0f);
	slipRatio = (tireV - forwardVelocity) / referenceSpeed;
	return slipRatio;
}

float TireDynamics::calculateSlipAngle(float carVx, float carVy, float yawRate, float rx, float ry, float steerAngle) {
	float wheelVx = carVx - (yawRate * ry);
	float wheelVy = carVy + (yawRate * rx);

	if (std::abs(wheelVx) < 0.1f) {
		this->slipAngle = 0.0f;
		return this->slipAngle;
	}

	float travelAngle = std::atan2(wheelVy, wheelVx);
	this->slipAngle = travelAngle - steerAngle;
	return this->slipAngle;
}

// --- Pacejka 模型計算 ---

float TireDynamics::calculateDynamicD(float load, float base_mu) {
	float b1 = -0.00005f; // 載重敏感度衰減率
	float mu = (b1 * load) + base_mu;
	mu = std::max(0.1f, mu); // 確保摩擦係數不會小於 0.1
	return load * mu;
}

float TireDynamics::calculatePacejkaForce(float input, float Fz, float B, float C, float D, float E) {
	float B_input = B * input;
	return D * std::sin(C * std::atan(B_input - E * (B_input - std::atan(B_input))));
}


// --- 主更新邏輯 ---
void TireDynamics::updateSlipState() {
	// 使用物件中的垂直載重（若未設置則使用預設值）
	float Fz_val = (this->Fz != 0.0f) ? this->Fz : 2000.0f; // 假設載重

	// 1. 計算理論縱向力 (使用 slipRatio 與 Fx 參數)
	float dynamic_D_x = calculateDynamicD(Fz_val, Fx_mu_base);
	this->Fx = calculatePacejkaForce(this->slipRatio, Fz_val, Fx_B, Fx_C, dynamic_D_x, Fx_E);

	// 2. 計算理論側向力 (使用 slipAngle 與 Fy 參數)
	float dynamic_D_y = calculateDynamicD(Fz_val, Fy_mu_base);
	float base_Fy = calculatePacejkaForce(this->slipAngle, Fz_val, Fy_B, Fy_C, dynamic_D_y, Fy_E);

	// 3. 【新增】計算外傾推力 (Camber Thrust)
	// 假設 camberAngle 已經由主程式傳入
	float camberThrust = Fz_val * this->camberStiffness * this->camberAngle;

	// 4. 將滑移力與外傾推力合併，作為「理論無限制側向力」
	this->Fy = base_Fy + camberThrust;

	// 5. 進行摩擦橢圓截斷
	updateTireForces();
}

void TireDynamics::updateTireForces() {

	float unconstrained_F_x = this->Fx;
	float unconstrained_F_y = this->Fy;

	float mu_x = 1.2f;
	float mu_y = 1.0f;

	float Fz_val = std::max(this->Fz, 1.0f);
	float max_Fx = mu_x * Fz_val;
	float max_Fy = mu_y * Fz_val;

	float ratio_x = unconstrained_F_x / max_Fx;
	float ratio_y = unconstrained_F_y / max_Fy;

	float rho = std::sqrt((ratio_x * ratio_x) + (ratio_y * ratio_y));

	if (rho > 1.0f) {
		this->Fx = (unconstrained_F_x / rho);
		this->Fy = (unconstrained_F_y / rho);
	}
	else {
		this->Fx = unconstrained_F_x;
		this->Fy = unconstrained_F_y;
	}
}
