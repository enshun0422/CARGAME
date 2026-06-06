#pragma once
#include "Vehicle.h"
#include <array>

class BMWM4GT3 : public Vehicle {

public:
	BMWM4GT3();
	std::string getName() override {
		return "BMW M4 GT3";
	}
};