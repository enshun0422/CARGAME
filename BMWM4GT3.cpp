#include "BMWM4GT3.h"

BMWM4GT3::BMWM4GT3() {
	engine = Engine(1000, 7200.0f, { {
		{1000, 320.0f},
		{2500, 460.0f},
		{3500, 590.0f},
		{4500, 650.0f},
		{5500, 700.0f},
		{6500, 680.0f},
		{7100, 620.0f}
	} });

	gearbox = Gearbox({ -3.20f, 0.0f, 3.25f, 2.29f, 1.70f, 1.35f, 1.12f, 0.93f },
		3.73f, 0.92f);

	aero = AeroDynamics(0.35f, 2.30f, 1.225f, 0.80f, 1.22f, 0.52f);

	setAntiRollBarStiffnessFront(140000.0f);
	setAntiRollBarStiffnessRear(65000.0f);
	setCgHeight(0.390f);
	setTotalMass(1285.0f);
	setAxleDistances(1.425f, 1.432f);
	setHalfTrackWidth(0.865f);
	setShiftRPM(7100.0f, 3500.0f);

	suspensions[0] = Suspension(0.2f, 125000.0f, 8500.0f);
	suspensions[1] = Suspension(0.2f, 125000.0f, 8500.0f);
	suspensions[2] = Suspension(0.2f, 140000.0f, 9500.0f);
	suspensions[3] = Suspension(0.2f, 140000.0f, 9500.0f);
}
