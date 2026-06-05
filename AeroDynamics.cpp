#include "AeroDynamics.h"

AeroDynamics::AeroDynamics() {
	dragCoefficient = 0.3f;
	frontalArea = 2.35f;
	airDensity = 1.225f;
	frontDownforceCoeff = 0.62f;
	rearDownforceCoeff = 0.88f;
	aeroCenterHeight = 0.52f;
}

AeroDynamics::AeroDynamics(float dragCoefficient, float frontalArea, float airDensity, float frontDownforceCoeff, float rearDownforceCoeff, float aeroCenterHeight) {
	this->dragCoefficient = dragCoefficient;
	this->frontalArea = frontalArea;
	this->airDensity = airDensity;
	this->frontDownforceCoeff = frontDownforceCoeff;
	this->rearDownforceCoeff = rearDownforceCoeff;
	this->aeroCenterHeight = aeroCenterHeight;
}

float AeroDynamics::calculateDragForce(float velocity) {
	// 判斷當前速度方向
	float sign = (velocity >= 0.0f) ? 1.0f : -1.0f;
	// 阻力必須帶有原本的速度方向
	return 0.5f * airDensity * dragCoefficient * frontalArea * velocity * velocity * sign;

}

float AeroDynamics::calculateFrontDownforce(float velocity) {
	return 0.5f * airDensity * frontDownforceCoeff * frontalArea * velocity * velocity;
}
float AeroDynamics::calculateRearDownforce(float velocity) {
	return 0.5f * airDensity * rearDownforceCoeff * frontalArea * velocity * velocity;
}

float AeroDynamics::calculateAeroPitchMoment(float velocity) {
	return 0.5f * airDensity * frontalArea * aeroCenterHeight * velocity * velocity;
}

