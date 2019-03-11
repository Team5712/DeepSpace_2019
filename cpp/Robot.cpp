/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
using namespace std;

void Robot::RobotInit()
{

	// drive_system = new frc::DifferentialDrive(*l_master, *r_master);
	int l_ids[3] = {1, 2, 3};
	int r_ids[3] = {4, 5, 6};
	drive = new GMDrive(l_ids, r_ids);

	shifter = new Solenoid(0);

	joystick_l = new frc::Joystick(0);
	joystick_r = new frc::Joystick(1);
	joystick_aux = new frc::Joystick(2);

	drive->SetSafetyEnabled(false);
	drive->initPID();

	compressor = new frc::Compressor(0);
	gyro = new AHRS(SPI::Port::kMXP, AHRS::kRawData, 200);
	gyro_correction = new GyroCorrection(gyro);
	climber = new Climber();
	elevator = new Elevator();
	intake = new Intake();
	passover = new Passover();

	vision_tracking = new Tracking();
	cout << "Robot init complete " << endl;
}

void Robot::handleDriverInput()
{

	if (joystick_l->GetRawButton(driver_buttons["trigger"]))
	{
		shifter->Set(false);
	}
	else
	{
		shifter->Set(true);
	}

	// VISION TAPE
	if (joystick_l->GetRawButton(driver_buttons["middle"]))
	{
		cout << "left middle" << endl;

		vision_tracking->setPipeline(0);

		// CARGO
	}
	else if (joystick_r->GetRawButton(driver_buttons["middle"]))
	{
		// cout << "right middle" << endl;
		// vision_tracking->setPipeline(1);
	}
	else if (joystick_l->GetRawButton(driver_buttons["left"]))
	{
		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(1, 0);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];
	}
	else if (joystick_l->GetRawButton(driver_buttons["right"]))
	{

		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(1, 1);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];
	}
	else if (joystick_r->GetRawButton(driver_buttons["left"]))
	{

		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(0, 0);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];
	}
	else if (joystick_r->GetRawButton(driver_buttons["right"]))
	{

		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(0, 1);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];
	}
	else if (joystick_l->GetRawButton(driver_buttons["trigger"]) && joystick_r->GetRawButton(driver_buttons["trigger"]))
	{
		drive->updateShifter(true);
	}
	else
	{
		drive->resetEncoders();
		drive->updateShifter(false);
	}
}

void Robot::handleAuxiliaryInput()
{

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//	Buttons for climbing switches on the toggle button: left_trigger
	//.................................................................................
	if (joystick_aux->GetRawButtonPressed(aux_buttons["left_bumper"]) && timer_climber.Get() > 30)
	{
		cout << "LEFT BUMPER" << endl;
		is_climbing_second = !is_climbing_second;
		is_climber_reset = false;
	}

	if (joystick_aux->GetRawButtonPressed(aux_buttons["right_bumper"]) && timer_climber.Get() > 30)
	{
		cout << "RIGHT BUMPER" << endl;
		is_climbing_third = !is_climbing_third;
		is_climber_reset = false;
	}

	if (joystick_aux->GetRawButton(aux_buttons["back"]))
	{
		if (!is_climber_reset)
		{
			if (resetClimber())
			{
				is_climber_reset = true;
			}
		}
	}

	climber->printEncoders();

	if (is_climbing_second || is_climbing_third)
	{
		compressor->Stop();
		handleClimb();
	}
	else
	{
		// if(joystick_aux->GetRawButtonPressed(aux_buttons["start"])) {
		// 	timer_climb_double_tap.Start();

		// 	// if(timer_climb_double_tap.Get() < ) {

		// 	// }
		// }

		// cout << passover->getPosition() << endl;
		// passover->setPosition(passover->setpoint_middle);
		// if (joystick_aux->GetRawAxis(aux_axis["left_y"]) < -0.85) {
		// 	passover->setPosition(passover->setpoint_front);
		// } else if(joystick_aux->GetRawAxis(aux_axis["left_y"]) > 0.85) {
		// 	passover->setPosition(passover->setpoint_back);
		// }
		// cout << "pot " << passover->getPosition() << endl;

		// if (joystick_aux->GetRawAxis(aux_axis["right_y"]) < -0.05)
		// {
		// 	left_intake->Set(joystick_aux->GetRawAxis(aux_axis["right_y"]));
		// 	right_intake->Set(joystick_aux->GetRawAxis(aux_axis["right_y"]));
		// }
		// else if (joystick_aux->GetRawAxis(aux_axis["right_y"]) > 0.05)
		// {
		// 	left_intake->Set(joystick_aux->GetRawAxis(aux_axis["right_y"]));
		// 	right_intake->Set(joystick_aux->GetRawAxis(aux_axis["right_y"]));
		// }
		// else
		// {
		// 	left_intake->Set(0);
		// 	right_intake->Set(0);
		// }

		if (joystick_aux->GetRawAxis(aux_axis["left_y"]) < -0.2)
		{
			//cout << joystick_aux->GetRawAxis(aux_axis["left_y"]) / 2 << endl;
			passover->setPower(joystick_aux->GetRawAxis(aux_axis["left_y"]) / 1.5);
		}
		else if (joystick_aux->GetRawAxis(aux_axis["left_y"]) > 0.2)
		{
			//cout << joystick_aux->GetRawAxis(aux_axis["left_y"]) / 2 << endl;cout << joystick_aux->GetRawAxis(aux_axis["left_y"]) / 2 << endl;
			passover->setPower(joystick_aux->GetRawAxis(aux_axis["left_y"]) / 1.5);
		}
		else
		{
			passover->setPower(0);
		}

		if (joystick_aux->GetPOV() == aux_buttons["pov_top"])
		{
			passover->sendPassoverFront();
		}
		else if (joystick_aux->GetPOV() == aux_buttons["pov_right"])
		{
		}
		else if (joystick_aux->GetPOV() == aux_buttons["pov_left"])
		{
			passover->sendPassoverMiddle();
		}
		else if (joystick_aux->GetPOV() == aux_buttons["pov_bottom"])
		{
			passover->sendPassoverBack();
		}

		if (joystick_aux->GetRawButton(aux_buttons["b"]))
		{
			elevator->setPosition(42);
		}
		else if (joystick_aux->GetRawButton(aux_buttons["y"]))
		{
			elevator->setPosition(77);
		}
		else if (joystick_aux->GetRawButton(aux_buttons["a"]) && elevator->getPosition() > 10)
		{
			elevator->setPosition(5);
		}
		else if (joystick_aux->GetRawButton(aux_buttons["x"]))
		{
			elevator->setPosition(15);
		}
		else
		{
			elevator->setPower(0);
		}

		if (joystick_aux->GetRawButton(aux_buttons["left_click"]) || joystick_r->GetRawButton(driver_buttons["middle"]))
		{
			intake->updateBeak(true);
		}
		else
		{
			intake->updateBeak(false);
		}
	}
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
	cout << "starting auto" << endl;
	gyro->ZeroYaw();
	gyro->ResetDisplacement();
	gyro_correction->zeroPitch();
	compressor->Start();
	gyro->ZeroYaw();
	drive->resetEncoders();
	climber->resetEncoders();
	is_climbing_second = false;
	is_climbing_third = false;
	is_climbing_complete = false;
	is_back_raised = false;
	is_front_raised = false;
	is_raising_back = false;
	is_raising_front = false;
}

void Robot::AutonomousPeriodic()
{

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Get adjustment values in order to make the robot drive more straight by adjusting
	// the cruise velocity in the pid loop according to the offset of the gyro
	//..................................................................................
	// vector<float> gyro_adjust_values = gyro_correction->adjustDrive();

	// drive->ConfigMotionCruiseVelocityDifferenceLeft(gyro_adjust_values[0]);
	// drive->ConfigMotionCruiseVelocityDifferenceRight(gyro_adjust_values[1]);
	// cout << "left " << drive->getLeftTicks() << endl;
	// cout << "right " << drive->getRightTicks() << endl;
	//drive->driveSetInches(24, 24);
	// cout << "pitch: " << gyro->GetPitch() - gyro_correction->pitch_zero << endl;

	// vector<float> f = <<gyro_correction->adjustClimb();

	// elevator->setPosition(75);
	// cout << joystick_aux->GetRawAxis(aux_axis["left_y"]) << endl;

	update();
}

void Robot::TeleopInit()
{
	timer_climber.Start();
	compressor->Start();
	gyro->ZeroYaw();
	drive->resetEncoders();
	climber->resetEncoders();
	is_climbing_second = false;
	is_climbing_third = false;
	is_climbing_complete = false;
	is_back_raised = false;
	is_front_raised = false;
	is_raising_back = false;
	is_raising_front = false;
}

void Robot::TeleopPeriodic()
{
	update();
	//cout << passover->getPosition() << endl;
	//cout << joystick_aux->GetRawAxis(aux_axis["left_y"]) << endl;
	//climber->printEncoders();
}

void Robot::update()
{

	passover->setPiston(true);

	handleDriverInput();
	handleAuxiliaryInput();

	// vision_tracking->logLimelightValue("thor");
	if (is_climbing_second || is_climbing_third)
	{
		drive->TankDrive(abs(joystick_l->GetRawAxis(1)), abs(joystick_r->GetRawAxis(1)), false);
	}
	else if (setpoint_l != 0 || setpoint_r != 0)
	{
		// cout << "going to setpoint" << endl;
		drive->driveSetTicks(setpoint_l, setpoint_r);
	}
	else
	{
		drive->TankDrive(joystick_l->GetRawAxis(1) + adjust_l, joystick_r->GetRawAxis(1) + adjust_r, false);
		resetDriveValues();
	}
}

void Robot::DisabledInit()
{
	cout << "I'm disabled :D" << endl;
}

void Robot::TestPeriodic()
{
}

void Robot::handleClimb()
{
	vector<float> values = gyro_correction->adjustClimb();

	// climb until we reach the max point, then stop lifting
	if (!is_climbing_complete && is_climbing_second)
	{
		if (climber->lowerClimberSecond(values[0], values[1]))
		{
			cout << "done climbing" << endl;
			is_climbing_complete = true;
		}
	}

	if (!is_climbing_complete && is_climbing_third)
	{
		if (climber->lowerClimberThird(values[0], values[1]))
		{
			cout << "done climbing" << endl;
			is_climbing_complete = true;
		}
	}

	// one we reach the top, sustain both sides until we specify not to with buttons
	if (is_climbing_complete)
	{

		climber->setLowRider(joystick_aux->GetRawAxis(aux_axis["left_y"]));

		// TODO make sure raising is done

		// is_raising_back = true;
		// is_raising_front = true;

		// the front is no longer lowered, start raising it
		if (is_raising_front)
		{
			if (!is_front_raised)
			{
				if (climber->raiseFront())
				{
					cout << "front finish raising!" << endl;
					is_front_raised = true;
					is_raising_front = false;
					climber->stopFront();
				}
			}
		}
		else if (is_front_raised)
		{
			climber->stopFront();
		}
		else
		{
			climber->sustainFront();
		}

		// the back is no longer lowered, start raising it
		if (is_raising_back)
		{
			if (!is_back_raised)
			{
				if (climber->raiseBack())
				{
					cout << "back finish raising!" << endl;
					is_back_raised = true;
					is_raising_back = false;
					climber->stopBack();
				}
			}
		}
		else if (is_back_raised)
		{
			//climber->setLowRider(-1);
			climber->stopBack();
		}
		else
		{
			climber->sustainBack();
		}

		if (joystick_aux->GetRawAxis(aux_axis["left_trigger"]) > 0.85)
		{
			cout << "left" << endl;
			is_raising_front = true;
		}

		if (joystick_aux->GetRawAxis(aux_axis["right_trigger"]) > 0.85)
		{
			cout << "right" << endl;
			is_raising_back = true;
		}
	}
}

bool Robot::resetClimber()
{
	is_climbing_second = false;
	is_climbing_third = false;
	is_climbing_complete = false;
	is_back_raised = false;
	is_front_raised = false;
	is_raising_back = false;
	is_raising_front = false;

	if (climber->raiseFront() && climber->raiseBack())
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Robot::resetDriveValues()
{
	adjust_l = 0;
	adjust_r = 0;

	setpoint_l = 0.0;
	setpoint_r = 0.0;
}

void Robot::setSetpoints(float l, float r)
{
	this->setpoint_l = l;
	this->setpoint_r = r;
}

void Robot::setAdjustments(float l, float r)
{
	this->setpoint_l = l;
	this->setpoint_r = r;
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
