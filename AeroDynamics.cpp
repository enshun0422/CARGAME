#include "AeroDynamics.h"

float AeroDynamics::calculateDragForce(float velocity) {
	return 0.5f * airDensity * dragCoefficient * frontalArea * velocity * velocity;
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

