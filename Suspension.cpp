#include "Suspension.h"

Suspension::Suspension(float rest, float k, float c)
    : restlength(rest),
    springRate(k),
    dampingRate(c),
    previousLength(rest),
    staticCamber(-0.0523f), // 【修正】預設約 -3 度的負外傾角 (八字腳)
    camberGain(0.15f)       // 【修正】每壓縮 1 公尺，增加 0.15 弧度的負外傾角
{
}

void Suspension::setRestLength(float rest) { restlength = rest; }
void Suspension::setSpringRate(float k) { springRate = k; }
void Suspension::setDampingRate(float c) { dampingRate = c; }
void Suspension::setStaticCamber(float camber) { staticCamber = camber; }
void Suspension::setCamberGain(float gain) { camberGain = gain; }
float Suspension::getCurrentCamber() const { return currentCamber; }

float Suspension::calculateForce(float currentLength, float dt) {
	float displacement = currentLength - restlength; // 彈簧位移 x (壓縮量)

    // 【修正】加入防護，避免 dt 過小導致除以零
    float safeDt = (dt > 0.0001f) ? dt : 0.0001f;
    float velocity = (currentLength - previousLength) / safeDt; // 速度 v
    velocity = std::clamp(velocity, -1.5f, 1.5f);

    previousLength = currentLength;

    float springForce = -springRate * displacement; // 彈簧力 F_s = -k * x
    float dampingForce = -dampingRate * velocity;   // 阻尼力 F_d = -c * v

    // 【關鍵修正】把車身靜態重量 (Static Weight) 加回去！
    // 否則車子預設會是「失重狀態」，避震器會完全被拉長
    float staticWeight = (1300.0f * 9.81f) / 4.0f; // 假設四輪均分

    float totalForce = staticWeight + springForce + dampingForce; // 總力 F

	return std::max(0.0f, totalForce); // 確保總力不為負值（只提供向上的力）
}
