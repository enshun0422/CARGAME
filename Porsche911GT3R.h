#pragma once
#include "Vehicle.h"
#include <array>

class Porsche911GT3R : public Vehicle {

public:
	Porsche911GT3R();
	std::string getName() override {
		return "Porsche 911 GT3 R";
	}
};
