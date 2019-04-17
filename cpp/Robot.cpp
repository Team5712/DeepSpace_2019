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

	int l_ids[3] = {2, 3, 1};
	int r_ids[3] = {5, 6, 4};
	// int l_ids[3] = {1, 2, 3};
	// int r_ids[3] = {4, 5, 6};
	drive = new GMDrive(l_ids, r_ids);

	joystick_l = new frc::Joystick(0);
	joystick_r = new frc::Joystick(1);
	joystick_aux = new frc::Joystick(3);
	joystick_button_board = new frc::Joystick(2);

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
	drive->resetEncoders();
	cout << "Robot init complete " << endl;
}

void Robot::handleDriverInput()
{

	// VISION TAPE
	if (joystick_l->GetRawButton(driver_buttons["middle"]))
	{
		cout << "left middle" << endl;

		// vision_tracking->setPipeline(0);

		// CARGO
	}
	else if (joystick_r->GetRawButton(driver_buttons["middle"]))
	{
		// cout << "right middle" << endl;
		// vision_tracking->setPipeline(1);
	}
	else if (joystick_l->GetRawButton(driver_buttons["left"]))
	{
		// vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(1, 0);

		// adjust_l = setpoint[0];
		// adjust_r = setpoint[1];
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
		cout << "right button pressed" << endl;

		vector<float> setpoint = vision_tracking->getTurnAdjustmentTicks(1, 0);

		setpoint_l = setpoint[0];
		setpoint_r = setpoint[1];
	}
	else if (joystick_l->GetRawButton(driver_buttons["trigger"]) && joystick_r->GetRawButton(driver_buttons["trigger"]))
	{
		drive->updateShifter(true);
		is_low = true;
	}
	else
	{
		is_low = false;
		drive->updateShifter(false);
	}
}

void Robot::handleAuxiliaryInput()
{

	// TODO: check this thing
	if (is_climbing_second || is_climbing_third)
	{
		compressor->Stop();
		climb();
	}
	else
	{

		// cargo mode
		if (!joystick_button_board->GetRawButton(10))
		{
			if (joystick_button_board->GetRawButton(3))
			{
				elevator->setPosition(elevator->positions.bottom);
				passover->sendPassoverFront();
			}
			else if (joystick_button_board->GetRawButton(4))
			{
				elevator->setPosition(elevator->positions.bottom + elevator->positions.cargo_offset);
				passover->sendPassoverBack();
			}
			else if (joystick_button_board->GetRawButton(5))
			{
				passover->sendPassoverMiddle();
			}
			else if (joystick_button_board->GetRawButton(6))
			{
				elevator->setPosition(elevator->positions.middle + 5);
				passover->sendPassoverFront();
			}
			else if (joystick_button_board->GetRawButton(7))
			{
				elevator->setPosition(elevator->positions.middle + elevator->positions.cargo_offset);
				passover->sendPassoverBack();
			}
			else if (joystick_button_board->GetRawButton(8))
			{
				// elevator->setPosition(elevator->positions.top);
				// passover->sendPassoverFrontCargo();
			}
			else if (joystick_button_board->GetRawButton(9))
			{
				elevator->setPosition(elevator->positions.top + elevator->positions.cargo_offset);
				passover->sendPassoverBack();
			}
			else
			{
				elevator->setPower(0);
				passover->setPower(0);
			}
		}
		// hatch mode
		else
		{
			if (joystick_button_board->GetRawButton(3))
			{
				elevator->setPosition(elevator->positions.bottom);
				passover->sendPassoverFront();
			}
			else if (joystick_button_board->GetRawButton(4))
			{
				elevator->setPosition(elevator->positions.bottom);
				passover->sendPassoverBack();
			}
			else if (joystick_button_board->GetRawButton(5))
			{
				passover->sendPassoverMiddle();
			}
			else if (joystick_button_board->GetRawButton(6))
			{
				elevator->setPosition(elevator->positions.middle);
				passover->sendPassoverFront();
			}
			else if (joystick_button_board->GetRawButton(7))
			{
				elevator->setPosition(elevator->positions.middle);
				passover->sendPassoverBack();
			}
			else if (joystick_button_board->GetRawButton(8))
			{
				elevator->setPosition(elevator->positions.top);
				passover->sendPassoverFront();
			}
			else if (joystick_button_board->GetRawButton(9))
			{
				elevator->setPosition(elevator->positions.top);
				passover->sendPassoverBack();
			}
			else
			{
				elevator->setPower(0);
				passover->setPower(0);
			}
		}

		// run intake outwards so we don't pick up a cargo while at hatch pick up
		if (joystick_button_board->GetRawButton(10) && elevator->getPosition() <= elevator->positions.bottom && passover->isPolarized())
		{
			intake->pushBall();
		}
		else
		{
			intake->stop();
		}

		// true is hatch
		if (joystick_button_board->GetRawButton(2) && joystick_button_board->GetRawButton(10))
		{
			passover->setPiston(false);
			intake->updateBeak(true);
		}
		else if (joystick_button_board->GetRawButton(10))
		{
			if (piston_timeout.Get() > 4)
			{
				passover->setPiston(true);
			}
			else
			{
				passover->setPiston(false);
			}
			intake->updateBeak(false);
		}
		else
		{
			passover->setPiston(false);
			intake->updateBeak(true);
		}

		// CARGO MODE
		if (joystick_button_board->GetRawButton(1) && !joystick_button_board->GetRawButton(10))
		{
			is_holding_ball = true;
			intake->intakeBall();
		}
		else if (joystick_button_board->GetRawButton(2) && !joystick_button_board->GetRawButton(10))
		{
			is_holding_ball = false;
			intake->releaseBall();
		}
		else if (!joystick_button_board->GetRawButton(10) && is_holding_ball)
		{
			intake->holdBall();
		}
		else
		{
			intake->stop();
		}
	}
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
	piston_timeout.Start();

	is_climbing_second = false;
	is_climbing_third = false;
	is_climbing_complete = false;
	is_back_raised = false;
	is_front_raised = false;
	is_raising_back = false;
	is_raising_front = false;

	auto_state = 0;

	cout << "starting auto" << endl;
	gyro->ZeroYaw();
	gyro->ResetDisplacement();
	gyro_correction->zeroPitch();
	compressor->Start();
	gyro->ZeroYaw();
	timer_climber.Start();
	drive->resetEncoders();
	climber->resetEncoders();
}

void Robot::AutonomousPeriodic()
{
	// turn ratio 0.4

	// if (joystick_r->GetRawAxis(2) < 0)
	// {
	// 	switch (auto_state)
	// 	{
	// 	case 0:
	// 		if (drive->setPositionTicks(55, 55))
	// 		{
	// 			auto_state++;
	// 			auto_transition_timer.Start();
	// 		}

	// 		break;
	// 	case 1:
	// 		if (auto_transition_timer.Get() > 1)
	// 		{
	// 			cout << "left " << drive->getLeftTicks() << " right " << drive->getRightTicks() << endl;
	// 			if (drive->setPositionTicks(-11.4, 11.4))
	// 			{
	// 				auto_state++;
	// 				cout << "turn complete" << endl;
	// 			}
	// 		}
	// 		else
	// 		{
	// 			drive->resetEncoders();
	// 		}

	// 		break;
	// 	case 2:
	// 		update();
	// 		break;
	// 	}
	// }
	// else if (joystick_l->GetRawAxis(2) < 0)
	// {
	// 	switch (auto_state)
	// 	{
	// 	case 0:
	// 		if (drive->setPositionTicks(55, 55))
	// 		{
	// 			auto_state++;
	// 			auto_transition_timer.Start();
	// 		}

	// 		break;
	// 	case 1:
	// 		if (auto_transition_timer.Get() > 1)
	// 		{
	// 			cout << "left " << drive->getLeftTicks() << " right " << drive->getRightTicks() << endl;
	// 			if (drive->setPositionTicks(11.4, -11.4))
	// 			{
	// 				auto_state++;
	// 				cout << "turn complete" << endl;
	// 			}
	// 		}
	// 		else
	// 		{
	// 			drive->resetEncoders();
	// 		}

	// 		break;

	// 	case 2:
	// 		update();
	// 		break;
	// 	}
	// }
	// else
	// {
	// 	update();
	// }


	drive->setPositionTicks(-7.8, 7.8);
	// drive->turnLeft();
	
	cout << "left " << drive->getLeftTicks() << " right " << drive->getRightTicks() << endl;
	// cout << "yaw " << gyro_correction->getYaw() << endl;

	drive->setPositionTicks(66, 66);

	// drive->TankDrive(0.2, -0.2, false);

	// update();
}

void Robot::TeleopInit()
{
	is_climbing_second = false;
	is_climbing_third = false;
	is_climbing_complete = false;
	is_back_raised = false;
	is_front_raised = false;
	is_raising_back = false;
	is_raising_front = false;
	// climb_state = CLIMB_STATES::READY;
	climber->resetEncoders();
}

void Robot::TeleopPeriodic()
{
	update();
}

void Robot::update()
{

	if (joystick_button_board->GetRawButton(12))
	{
		if (joystick_button_board->GetRawButton(11))
		{
			is_climbing_second = true;
		}
		else
		{
			is_climbing_third = true;
		}
	}
	else
	{
		resetClimbValues();
	}

	handleDriverInput();
	handleAuxiliaryInput();

	// vision_tracking->logLimelightValue("thor");
	if (is_climbing_second || is_climbing_third)
	{
		drive->TankDrive(abs(joystick_l->GetRawAxis(1)), abs(joystick_r->GetRawAxis(1)), false);
	}
	else if (setpoint_l != 0 || setpoint_r != 0)
	{
		cout << "going to setpoint L: " << setpoint_l << " right " << setpoint_r << endl;
		drive->driveSetTicks(setpoint_l, setpoint_r);
	}
	else
	{

		if (joystick_l->GetRawButton(driver_buttons["middle"]))
		{
			drive->TankDrive(joystick_l->GetRawAxis(1) + adjust_l, joystick_l->GetRawAxis(1) + adjust_r, false);
		}
		else
		{

			// cout << "left " << adjust_l << " right " << adjust_r << endl;
			if (is_low)
			{
				drive->TankDrive(joystick_l->GetRawAxis(1) + adjust_l, joystick_r->GetRawAxis(1) + adjust_r, false);
			}
			else
			{
				drive->TankDrive((1 * joystick_l->GetRawAxis(1)) + adjust_l, (1 * joystick_r->GetRawAxis(1)) + adjust_r, false);
			}
		}

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

void Robot::climb()
{

	// TODO: STATES CHECK GYRO FOR CLIMB
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

	if (!is_climbing_second && !is_climbing_third)
	{
		climber->setLiftPower(0, 0);
	}

	// one we reach the top, sustain both sides until we specify not to with buttons
	if (is_climbing_complete)
	{

		if (!is_front_raised && !is_back_raised)
		{
			if (joystick_button_board->GetRawAxis(2) < 0)
			{
				climber->setLowRider(-0.25);
			}
			else
			{
				// cout << "joy " << -abs(joystick_button_board->GetRawAxis(2)) - 0.25 << endl;
				climber->setLowRider(-abs(joystick_button_board->GetRawAxis(2)) - 0.25 + (joystick_button_board->GetRawAxis(2) * 0.24));
			}
		}
		else
		{
			climber->setLowRider(0);
		}

		// TODO make sure raising is done

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

		if (joystick_button_board->GetRawButton(14))
		{
			cout << "RAISING FRONT" << endl;
			is_raising_front = true;
		}

		if (joystick_button_board->GetRawButton(13))
		{
			cout << "RAISING BACK" << endl;
			is_raising_back = true;
		}
	}
}

bool Robot::resetClimbValues()
{
	is_climbing_third = false;
	is_climbing_second = false;
	is_climbing_complete = false;
	is_raising_front = false;
	is_raising_back = false;
	is_front_raised = false;
	is_back_raised = false;
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
