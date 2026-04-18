#include "AeroDynamics.h"

float AreoDynamics::calculateDragForce(float velocity) {
	return 0.5f * airDensity * dragCoefficient * frontalArea * velocity * velocity;
}