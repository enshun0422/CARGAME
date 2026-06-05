#pragma once
#include <iostream>
#include <cmath>

class AeroDynamics {
	private:
		//M4 GT3 EVO 參考數據
		float dragCoefficient = 0.3f; // 前阻力係數
		float frontalArea = 2.35f; // 車輛正面積 (m^2)
		float airDensity = 1.225f; // 空氣密度 (kg/m^3)

		float frontDownforceCoeff = 0.62f; // 前下壓力係數
		float rearDownforceCoeff = 0.88f; // 後下壓力係數

		float  aeroCenterHeight = 0.52f; // 空氣動力中心高度 (m),高速車身後力矩
	public:
		AeroDynamics();
		AeroDynamics(float, float, float, float, float, float);
		float calculateDragForce(float velocity);

		float calculateFrontDownforce(float velocity);
		float calculateRearDownforce(float velocity);

		float calculateAeroPitchMoment(float velocity);

};