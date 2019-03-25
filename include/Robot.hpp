
#pragma once

#include <cmath>
#include <iostream>
#include <map>
#include <string>

#include "frc/WPILib.h"
#include <frc/IterativeRobot.h>
#include "AHRS.h"
#include "ctre/Phoenix.h"

#include <control_systems/GMDrive.h>
#include <control_systems/GyroCorrection.h>
#include <control_systems/Climber.h>
#include <control_systems/Elevator.h>
#include <control_systems/Intake.h>
#include <control_systems/Passover.h>

#include "vision/Tracking.h"

using namespace std;

class Robot : public frc::IterativeRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void DisabledInit() override;

  void update();

  void setSetpoints(float, float);
  void setAdjustments(float, float);

private:

  AHRS *gyro;
  int count = 0;

  float adjust_l = 0.0;
  float adjust_r = 0.0;

  float setpoint_l = 0.0;
  float setpoint_r = 0.0;

  frc::Joystick *joystick_l;
  frc::Joystick *joystick_r;
  frc::Joystick *joystick_aux;
  frc::Compressor *compressor;

  Tracking *vision_tracking;
  GyroCorrection *gyro_correction;
  Climber *climber;
  Elevator *elevator;
  Passover *passover;
  Intake *intake;

  Solenoid *shifter;


  Timer low_rider_timer;

  Timer timer_climber;

  float low_rider_delay = 1;

  const float ratio = (515 / (2 * 2 * M_PI));

  float intake_power = 0.5;

  bool is_climbing_third = false;
  bool is_climbing_second = false;
  // are we at our max height
  bool is_climbing_complete = false;

  bool is_raising_front = false;
  bool is_raising_back = false;

  bool is_front_raised = false;
  bool is_back_raised = false;

  bool is_climber_reset = false;

  bool is_pot_zero = false;

  
  Timer timer_climb_double_tap;

  float climb_double_tap_time_frame;

  // TODO: implement this on the right side and fix the stuff
  // frc::DifferentialDrive *drive_system;
  GMDrive *drive;

  map<string, int> driver_buttons = {
	  {"trigger", 1},
	  {"left", 2},
	  {"right", 3},
	  {"middle", 4},
  };

  // input mapping for aux controller
  map<string, int> aux_buttons = {
	    {"a", 1}, 
  	  {"b", 2},
  	  {"x", 3},
  	  {"y", 4},
  	  {"left_bumper", 5},
  	  {"right_bumper", 6},
  	  {"back", 7},
  	  {"start", 8},
  	  {"left_click", 9},
  	  {"right_click", 10},
      {"pov_none", -1},
      {"pov_top", 0},
      {"pov_top_right", 45},
      {"pov_right", 90},
      {"pov_bottom_right", 135},
      {"pov_bottom", 180},
      {"pov_bottom_left", 225},
      {"pov_left", 270},
      {"pov_top_left", 315}, 
  };


  map<string, int> driver_axis = {
  	  {"trigger", 1},
  	  {"left", 2},
  	  {"right", 3},
  	  {"middle", 4},
    };

  map<string, int> aux_axis = {
	  {"left_x", 0},
	  {"left_y", 1},
	  {"left_trigger", 2},
	  {"right_trigger", 3},
    {"right_x", 4},
    {"right_y", 5},
  };

  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Input methods for each respective joystick to update their respective systems
  //...............................................................................
  void handleDriverInput();
  void handleAuxiliaryInput();
  void handleClimb();
  bool resetClimber();
  void resetDriveValues();
};
