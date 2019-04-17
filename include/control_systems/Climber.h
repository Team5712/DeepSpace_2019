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
  float front_offset = 0;

  // the range of ticks we need to be at for each setpoint before we stop
  // float error_margin = 35;

  float climb_speed = 0.75;

  // float position_max = 18000;
  // float position_min = 10;

  float raise_speed = 0.40;

  float climb_max = 20;

  // 18000
  float third_position = 18000;
  float second_position = 7500;

public:
  Climber();
  void setLiftPower(float, float);
  // lifts the climber to the top with the given correction valuess

  // lifts the climber to the top with the given correction valuess
  bool lowerClimberThird(float, float);
  bool lowerClimberSecond(float, float);

  void sustainLift();
  void sustainFront();
  void sustainBack();

  void printEncoders();
  bool raiseFront();
  bool raiseBack();

  bool setPositions(float, float);

  float getFrontPosition();
  float getBackPosition();

  void stop();
  void stopFront();
  void stopBack();

  void resetEncoders();

  void setLowRider(float);

  struct front_climber_pid
  {
    float prev_error = 0;
    // 125
    float Kminoutput = -0.80;
    float Kmaxoutput = 0.80;
    float i_state = 0;
    float Kp = 0.0007;
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
    float Kd = 0.01;
    float Kf = 0.0;
    float i_zone = 0.0;
  } back_pid;

};

#endif /* SRC_CONTROL_SYSTEMS_CLIMBING_H_ */
