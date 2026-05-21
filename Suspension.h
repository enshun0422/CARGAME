#pragma once
#include <iostream>
#include <algorithm>
#include <cmath>

class Suspension {
private:
	float restlength;
	float springRate;
	float dampingRate;

	float staticCamber; // 靜態外傾角 (弧度)
	float camberGain; // 外傾角增益 (弧度 / 公尺)
	float previousLength;
	
	float currentCamber = 0.0f;
public:
	void setRestLength(float rest);
	void setSpringRate(float k);
	void setDampingRate(float c);
	void setStaticCamber(float camber);
	void setCamberGain(float gain);
	float getCurrentCamber() const;

	Suspension(float rest, float k, float c);

	float calculateForce(float currentLength, float dt);


};
