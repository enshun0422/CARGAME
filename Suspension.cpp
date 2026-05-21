#include "Suspension.h"

Suspension::Suspension(float rest, float k, float c) : restlength(rest), springRate(k), dampingRate(c), previousLength(rest) {}

void Suspension::setRestLength(float rest) { restlength = rest; }
void Suspension::setSpringRate(float k) { springRate = k; }
void Suspension::setDampingRate(float c) { dampingRate = c; }
void Suspension::setStaticCamber(float camber) { staticCamber = camber; }
void Suspension::setCamberGain(float gain) { camberGain = gain; }
float Suspension::getCurrentCamber() const { return currentCamber; }

float Suspension::calculateForce(float currentLength, float dt) {
	float displacement = currentLength - restlength; // 彈簧位移 x (壓縮量)

	currentCamber = staticCamber + (displacement * camberGain);

	float velocity = (currentLength - previousLength) / dt; // 速度 v
	previousLength = currentLength;

	float springForce = -springRate * displacement; // 彈簧力 F_s = -k * x
	float dampingForce = -dampingRate * velocity; // 阻尼力 F_d = -c * v

	float totalForce = springForce + dampingForce; // 總力 F = F_s + F_d

	return std::max(0.0f, totalForce); // 確保總力不為負值（只提供向上的力）
}
