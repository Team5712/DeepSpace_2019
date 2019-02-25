#ifndef SRC_CONTROL_SYSTEMS_CLIMBER_H_
#define SRC_CONTROL_SYSTEMS_CLIMBER_H_

#include <cmath>
#include <iostream>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "AHRS.h"


using namespace std;

/**
 * this class will implement many of the basic functionalities of our drive system
 * into one convenient class with drive and turning options
 * 
 */
class Climber
{
	WPI_TalonSRX *front_lift;
	WPI_TalonSRX *back_lift;

	WPI_VictorSPX *low_rider;

    float climb_sustain = -0.20;
    float back_offset = 0.0;
    // -0.098

    float climb_speed = 0.75;
  //.45
    float climb_third = 18000;
    float climb_second = 7500;
    // 18000
    float climb_max = 20;

    // speed we raise the legs back up
    float raise_speed = 0.35;


  public:
    Climber();
    void setLiftPower(float, float);
    // lifts the climber to the top with the given correction valuess
    bool lowerClimberThird(float, float);
    bool lowerClimberSecond(float, float);

    void sustainLift();
    void sustainFront();
    void sustainBack();

    bool raiseFront();
    bool raiseBack();

    void stop();
    void stopFront();
    void stopBack();

    void resetEncoders();

    void setLowRider(float);

  };

#endif /* SRC_CONTROL_SYSTEMS_CLIMBING_H_ */
