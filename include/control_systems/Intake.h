#ifndef SRC_CONTROL_SYSTEMS_INTAKE_H_
#define SRC_CONTROL_SYSTEMS_INTAKE_H_

#include <cmath>
#include <iostream>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "AHRS.h"


using namespace std;

class Intake
{
    WPI_VictorSPX *intake;

    float intake_power = 0;

    public:
    Intake();
    void setIntakePower(float);
    void stop();
};

#endif /* SRC_CONTROL_SYSTEMS_INTAKE_H_ */
