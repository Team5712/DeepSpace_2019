#ifndef SRC_CONTROL_SYSTEMS_INTAKE_H_
#define SRC_CONTROL_SYSTEMS_INTAKE_H_

#include <cmath>
#include <iostream>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "Spark.h"

using namespace std;

class Intake
{
    WPI_VictorSPX *intake;
    frc::Solenoid *beak;
    

    float intake_power = -0.6;
    float release_power = 0.8;

    public:
    Intake();
    void intakeBall();
    void releaseBall();
    void stopIntake();
    void setIntakePower(float);
    void updateBeak(bool);
    void stop();
};

#endif /* SRC_CONTROL_SYSTEMS_INTAKE_H_ */
