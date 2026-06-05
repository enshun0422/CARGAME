#include "AMGGT3.h"

AMGGT3::AMGGT3() {
	engine = Engine(1000, 7600.0f,{{
		{1000, 300.0f},
		{2500, 380.0f},
		{3800, 480.0f},
		{4800, 600.0f}, 
		{5300, 650.0f},
		{6500, 600.0f},
		{7100, 550.0f}
	} });

	gearbox = Gearbox({ -3.0f, 0.0f, 3.10f, 2.30f, 1.75f, 1.38f, 1.10f, 0.82f }, 
						3.70f, 0.92f);

	aero = AeroDynamics(0.36f, 2.25f, 1.225f, 0.78f, 1.15f, 0.48f);

	setAntiRollBarStiffnessFront(150000.0f);
	setAntiRollBarStiffnessRear(50000.0f);
	setCgHeight(0.385f);
	setTotalMass(1285.0f);
	setAxleDistances(1.367f, 1.262f);
	setHalfTrackWidth(0.862f);

	suspensions[0] = Suspension(0.2f, 130000.0f, 8500.0f);
	suspensions[1] = Suspension(0.2f, 130000.0f, 8500.0f);
	suspensions[2] = Suspension(0.2f, 145000.0f, 9500.0f);
	suspensions[3] = Suspension(0.2f, 145000.0f, 9500.0f);

}