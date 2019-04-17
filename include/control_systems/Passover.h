#ifndef SRC_CONTROL_SYSTEMS_PASSOVER_H_
#define SRC_CONTROL_SYSTEMS_PASSOVER_H_

#include <cmath>
#include <iostream>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/SparkMax.h"
#include "AHRS.h"

using namespace std;

/**
 * 
 * Negative goes back
 * Positive goes forward
 */
class Passover
{
  AnalogPotentiometer *pot;

  rev::SparkMax *motor;

  float max_height = 78;

public:
  Passover();

  float getPosition();
  bool isFront();
  bool isBack();
  void printPotentiometer();

  void setPosition(float);

  void setPower(float);

  void sendPassoverBack();
  void sendPassoverFront();
  void sendPassoverMiddle();
  void sendPassoverFrontCargo();

  // checks if passover is near front or back
  bool isPolarized();

  void setPiston(bool);

  // TODO: COMP front FRONT 21 BACK 14
  // PRACTICE FRONT: 5.1 BACK 25.5
  float max_front = 64.8;  
  float max_back = 52.5;
  // front 56.9, back 45.8

  float setpoint_front = 64.8; //56
  float setpoint_back = 52.5; //45
  float setpoint_middle = 57.8; //57
  float setpoint_front_cargo = 14;

  float setpoint_angle = 0.0;

  float polar_range = 1;

  Solenoid *piston;

  struct passover_pid
  {
    float prev_error = 0;
    // TODO: .68 max
    float Kminoutput = -0.50;
    float Kmaxoutput = 0.50;
    float i_state = 0;
    float Kp = 0.155;
    float Ki = 0;
    float Kd = 0.0;
    float Kf = 0.0;
    float i_zone = 0.0;
  } pid;
};
#endif /*  SRC_CONTROL_SYSTEMS_PASSOVER_H_ */
