#include "AeroDynamics.h"

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

