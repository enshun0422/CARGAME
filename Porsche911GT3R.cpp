#include "Porsche911GT3R.h"

Porsche911GT3R::Porsche911GT3R() {
	engine = Engine(1000, 9000.0f, { {
		{1000, 260.0f},
		{2500, 340.0f},
		{4000, 410.0f},
		{5500, 455.0f},
		{7000, 470.0f},
		{8000, 465.0f},
		{8800, 430.0f}
	} });

	gearbox = Gearbox({ -3.45f, 0.0f, 3.75f, 2.38f, 1.72f, 1.34f, 1.11f, 0.96f },
		3.89f, 0.92f);

	aero = AeroDynamics(0.34f, 2.05f, 1.225f, 0.82f, 1.25f, 0.55f);

	setAntiRollBarStiffnessFront(120000.0f);
	setAntiRollBarStiffnessRear(85000.0f);
	setCgHeight(0.380f);
	setTotalMass(1250.0f);
	setAxleDistances(1.470f, 0.989f);
	setHalfTrackWidth(0.840f);
	setShiftRPM(8700.0f, 4000.0f);

	suspensions[0] = Suspension(0.2f, 120000.0f, 8000.0f);
	suspensions[1] = Suspension(0.2f, 120000.0f, 8000.0f);
	suspensions[2] = Suspension(0.2f, 155000.0f, 10000.0f);
	suspensions[3] = Suspension(0.2f, 155000.0f, 10000.0f);
}
