#pragma once
#include "Tire.h"
#include <iostream>
#include <cmath>

class TireDynamics {
	private:
		float Fx = 0.0f; // 縱向力 (摩擦力) Longitudinal Force
		float Fy = 0.0f; // 橫向力 (側向力) Lateral Force
		float Fz = 0.0f; // 垂直載重 Vertical Load

		float Mz = 0.0f; // 回正力矩 Self-Aligning Torque
		float RR = 0.0f; // 滾動阻力 Rolling Resistance

		float slipRatio = 0.0f; // 打滑比率 Slip Ratio
		float slipAngle = 0.0f; // 滑移角 Slip Angle

		// --- 縱向力 (Fx) Pacejka 參數 ---
		static constexpr float Fx_B = 10.0f;
		static constexpr float Fx_C = 1.65f;
		static constexpr float Fx_E = 0.97f;
		float Fx_mu_base = 1.45f; // 熱熔胎的縱向極限

		// --- 側向力 (Fy) Pacejka 參數 ---
		static constexpr float Fy_B = 12.0f; // 側向初始攀升通常更快 (Cornering Stiffness 高)
		static constexpr float Fy_C = 1.35f; // 側向頂峰較平緩
		static constexpr float Fy_E = -0.5f; // 注意：真實側向 E 值有時會是負數，以匹配真實衰減曲線
		float Fy_mu_base = 1.40f; // 熱熔胎的側向極限
		float camberAngle = 0.0f;           // 當前外傾角 (弧度)
		float camberStiffness = 0.05f;      // 外傾剛性 (需要調校的常數)

	public:
		Tire tire;
		void setLongitudinalForce(float fx);
		void setLateralForce(float fy);
		void setVerticalLoad(float load);//Fz


		void setCamberAngle(float camber);  // 設定外傾角的函式
		float getLongitudinalForce() const; // 縱向力 (摩擦力)
		float getLateralForce() const; // 橫向力 (側向力)

		float getAngularVel(Tire tire) const;

		// --- 物理計算函式 ---
		float calculslipRatio(float forwardVelocity);
		float calculateSlipAngle(float carVx, float carVy, float yawRate, float rx, float ry, float steerAngle);

		// Pacejka 核心數學模型
		float calculateDynamicD(float load, float base_mu);
		static float calculatePacejkaForce(float s, float Fz, float B, float C, float D, float E);
		
		// 每幀更新狀態
		void updateSlipState();
		void updateTireForces();
};