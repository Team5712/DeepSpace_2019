#ifndef SRC_CONTROL_SYSTEMS_INTAKE_H_
#define SRC_CONTROL_SYSTEMS_INTAKE_H_

#include <cmath>
#include <iostream>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/SparkMax.h"
#include "AHRS.h"
#include "Spark.h"

using namespace std;

class Intake
{
    rev::SparkMax *intake;
    frc::Solenoid *beak;
    
    float push_power = 0.3;
    float intake_power = -1;
    float release_power = 1.0;
    float hold_power = -0.2;

    public:
    Intake();
    void intakeBall();
    void releaseBall();
    void stopIntake();
    void pushBall();
    void holdBall();
    void setIntakePower(float);
    void updateBeak(bool);
    void stop();
};

#endif /* SRC_CONTROL_SYSTEMS_INTAKE_H_ */
