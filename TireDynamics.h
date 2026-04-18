#pragma once
#include "Wheel.h"
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

		static constexpr float Default_B = 10.0f; // 剛性因子 Stiffness Factor
		static constexpr float Default_C = 1.5f; // 形狀因子 Shape Factor
		static constexpr float Default_D = 1.0f; // 峰值因子 Peak Factor
		static constexpr float Default_E = 0.97; // 曲率因子 Curvature Factor
	public:
		Wheel wheel;
		void setLongitudinalForce(float fx);
		void setLateralForce(float fy);
		void setVerticalLoad(float load);//Fz

		static float calculatePacejkaFx(float s, float Fz, float B, float C, float D, float E);
		float calculslipRatio(float forwardVelocity);

		void updateTireForces();
		void updateSlipState();

		float getLongitudinalForce() const;

};