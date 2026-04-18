#pragma once
#include <iostream>
#include <cmath>

class AreoDynamics {
	private:
		float dragCoefficient = 0.3f; // 空氣阻力係數
		float frontalArea = 2.2f; // 車輛正面積 (m^2)
		float airDensity = 1.225f; // 空氣密度 (kg/m^3)
	public:
		float calculateDragForce(float velocity);
};