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

  float climb_sustain = -0.240;
  float front_offset = 0; //0.055
  // -0.098

  float climb_speed = 0.75;
  // 18450
  float climb_third = 18000;
  float climb_second = 7500;
  // 18000
  float climb_max = 20;

  // speed we raise the legs back up
  float raise_speed = 0.40;

public:
  Climber();
  void setLiftPower(float, float);
  // lifts the climber to the top with the given correction valuess
  bool lowerClimberThird(float, float);
  bool lowerClimberSecond(float, float);

  void setPositions(float, float);

  void sustainLift();
  void sustainFront();
  void sustainBack();

  void printEncoders();
  bool raiseFront();
  bool raiseBack();

  void stop();
  void stopFront();
  void stopBack();

  void resetEncoders();

  void setLowRider(float);

  struct front_climber_pid
  {
    float prev_error = 0;
    float Kminoutput = -0.125;
    float Kmaxoutput = 0.70;
    float i_state = 0;
    float Kp = 0.065;
    float Ki = 0.0;
    float Kd = 0.0;
    float Kf = 0.0;
    float i_zone = 0.0;
  } front_pid;

  struct back_climber_pid
  {
    float prev_error = 0;
    float Kminoutput = -0.125;
    float Kmaxoutput = 0.70;
    float i_state = 0;
    float Kp = 0.065;
    float Ki = 0.0;
    float Kd = 0.0;
    float Kf = 0.0;
    float i_zone = 0.0;
  } back_pid;
};

#endif /* SRC_CONTROL_SYSTEMS_CLIMBING_H_ */
