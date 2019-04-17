#ifndef SRC_CONTROL_SYSTEMS_ELEVATOR_H_
#define SRC_CONTROL_SYSTEMS_ELEVATOR_H_

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
class Elevator
{
    rev::CANSparkMax *elevator_master;
    rev::CANSparkMax *elevator_slave;

    AnalogPotentiometer *pot;

    float max_height = 78;
    
  public:
    Elevator();

    float getPosition();

    void setPosition(float);

    void setPower(float);

    struct ElevatorPositions {
      const float bottom = 5;
      const float middle = 42;
      const float top = 74;
      const float cargo_offset = -5;
    } positions;

    struct elevator_pid {
      float prev_error = 0;
      float Kminoutput = -0.205;
      float Kmaxoutput = 0.65;
      float i_state = 0;
      float Kp = 0.065;
      float Ki = 0.0;
      float Kd = 0.0;
      float Kf = 0.0;
      float i_zone = 0.0;
    } pid; 

};

#endif /* SRC_CONTROL_SYSTEMS_ELEVATOR_H_ */
