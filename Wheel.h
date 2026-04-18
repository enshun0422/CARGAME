#pragma once
#include <iostream>
#include <cmath>



class Wheel {
private:
    float radius = 0.33f;    // 18吋胎半徑約 0.33公尺
	float inertia = 1.2f;    // 輪胎轉動慣量 (kg.m^2)
	float dampingCoeff = 0.1f; // 阻尼係數

    float angularVel = 0.0f; // 角速度 (rad/s)
	float rotationAngle = 0.0f; // 角位置 (rad)
	float transformOffset = 0.0f; // 懸吊產生的垂直偏移量 (m)
	
	float roadFriction = 0.0f; // 摩擦力
	float breakingForce = 0.0f; // 制動力
	
	float tangentialVelocity = 0.0f; // 輪胎切線速度 (m/s) (v_tire)

public:
	void setFriction(float friction);
	void setBreakingForce(float breakingForce);
	
	void update(float driveTorque, float dt); // 摩擦力也會影響輪胎轉動

	float calculateAngularAcceleration(float driveTorque); // 根據驅動扭矩和時間步長更新角速度
	void integrateRotation(float driveTorque, float dt); // 更新角速度和位置

	

	float getNetTorque(float driveTorque) const;
	float getTireVelocity() const ;// 輪速轉車速
	
};
