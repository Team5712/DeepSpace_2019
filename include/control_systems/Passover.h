#ifndef SRC_CONTROL_SYSTEMS_PASSOVER_H_
#define SRC_CONTROL_SYSTEMS_PASSOVER_H_

#include <cmath>
#include <iostream>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
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

  PWMVictorSPX *motor;

  float max_height = 78;

public:
  Passover();

  float getPosition();

  void setPosition(float);

  void setPower(float);

  void sendPassoverBack();
  void sendPassoverFront();
  void sendPassoverMiddle();

  void setPiston(bool);

  // 15.5, 27.5
  float max_front = 57.5; //15.5
  float max_back = 45;    //26.5

  float setpoint_front = 56;
  float setpoint_back = 45;
  float setpoint_middle = 50;

  float setpoint_angle = 0.0;

  Solenoid *piston;

  struct passover_pid
  {
    float prev_error = 0;
    float Kminoutput = -0.68;
    float Kmaxoutput = 0.68;
    float i_state = 0.20;
    float Kp = 0.230;
    float Ki = 0.001;
    float Kd = 1.0;
    float Kf = 0.0;
    float i_zone = 0.0;
  } pid;
};
#endif /*  SRC_CONTROL_SYSTEMS_PASSOVER_H_ */
