#pragma once

class Wheel {
private:
    float radius = 0.33f;    // 18吋胎半徑約 0.33公尺
    float angularVel = 0.0f; // 角速度 (rad/s)
    float inertia = 1.2f;    // 輪胎轉動慣量
	float slipRatio ; // 打滑比率
	float Fx = 0.0f; // 縱向力 (摩擦力)
	
public:
    void update(float driveTorque, float dt); // 摩擦力也會影響輪胎轉動
    float calculslipRatio(float forwardVelocity); // 傳入車速
	float getTireVelocity();// 輪速轉車速
	float updateSlipState();// 根據打滑比率更新輪胎狀態，計算摩擦力
	float getSlipRatio() { return slipRatio; } // 提供外部讀取打滑比率的接口

	//備用版本，使用 Pacejka Magic Formula 計算縱向力 Fx，提供更真實的輪胎行為模擬。
	float calculatePacejkaFx(float s, float Fz);// 計算縱向力 Fx (Magic Formula 簡化版)
};
