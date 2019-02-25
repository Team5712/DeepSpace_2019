/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
using namespace std;

void Robot::RobotInit() {


	// drive_system = new frc::DifferentialDrive(*l_master, *r_master);
	int l_ids[3] = {1, 2, 3};
	int r_ids[3] = {4, 5, 6};
	drive = new GMDrive(l_ids, r_ids);

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

	vision_tracking = new Tracking();
	cout << "Robot init complete " << endl;
}

void Robot::handleDriverInput() {

	// // VISION TAPE
	// if(joystick_l->GetRawButton(driver_buttons["middle"])) {
	// 	cout << "left middle" << endl;

	// 	vector<float> adjust = vision_tracking->getTurnAdjustmentPercents(0);
	// 	adjust_l = adjust[0];
	// 	adjust_r = adjust[1];

	// // track cargo
	// } else if(joystick_r->GetRawButton(driver_buttons["middle"])) {
	// 	cout << "right middle" << endl;

	// 	vector<float> adjust = vision_tracking->getTurnAdjustmentPercents(1);

	// 	adjust_l = adjust[0];
	// 	adjust_r = adjust[1];

	// 	// TODO: wait based on variable in tracking.hpp instead of doing it all the time
	// 	// drive to setpoint from given target
	// } else if(joystick_l->GetRawButtonPressed(driver_buttons["trigger"])) {
	// 	cout << "left trigger" << endl;

	// 	vector<float> setpoint = vision_tracking->getTurnAdjustmentTicks(0);

	// 	setpoint_l = setpoint[0];
	// 	setpoint_r = setpoint[1];

	// } else if(joystick_l->GetRawButton(driver_buttons["trigger"])) {
	// 	if(abs(drive->getLeftTicks() - setpoint_l) < 40 || abs(drive->getRightTicks() - setpoint_r) < 40
	// 			&& (setpoint_l <= 0 || setpoint_r <= 0)) {
	// 		cout << "done turning" << endl;
	// 		drive->resetEncoders();
	// 		//cout << "l " << abs(drive->getLeftTicks() - setpoint_l) << "r " << abs(drive->getRightTicks() - setpoint_r) << endl;
	// 		float setpoint = vision_tracking->getDriveFromSetPointTicks(12);
	// 		setpoint_l = setpoint;
	// 		setpoint_r = setpoint;
	// 		cout << "DISTANCE SET " << setpoint << endl;
	// 	}

	// 	if((abs(drive->getLeftTicks() - setpoint_l) < 60 || abs(drive->getRightTicks() - setpoint_r) < 60) && (setpoint_l > 0 && setpoint_r > 0)) {
	// 		cout << "done driveing " << setpoint_l << " " << setpoint_r << endl;
	// 		setpoint_l = 0;
	// 		setpoint_r = 0;
	// 	}
	// } else if(joystick_l->GetRawButtonReleased(driver_buttons["trigger"])) {
	// 	cout << "stop" << endl;
	// 	resetDriveValues();
	// } else if (!joystick_l->GetRawButton(driver_buttons["trigger"])) {
	// 	drive->resetEncoders();
	// } 

	
	// VISION TAPE
	if(joystick_l->GetRawButton(driver_buttons["middle"])) {
		cout << "left middle" << endl;

		vision_tracking->setPipeline(0);

	// CARGO
	} else if(joystick_r->GetRawButton(driver_buttons["middle"])) {
		cout << "right middle" << endl;
		vision_tracking->setPipeline(1);

	} else if(joystick_l->GetRawButton(driver_buttons["left"])) {
		cout << "left trigger" << endl;

		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(1, 0);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];

	} else if(joystick_l->GetRawButton(driver_buttons["right"])) {

		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(1, 1);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];

	} else if(joystick_r->GetRawButton(driver_buttons["left"])) {
		
		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(0, 0);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];

	} else if (joystick_r->GetRawButton(driver_buttons["right"])) {
		
		vector<float> setpoint = vision_tracking->getTurnAdjustmentPercents(0, 1);

		adjust_l = setpoint[0];
		adjust_r = setpoint[1];
	} else {
		drive->resetEncoders();
	}
}

void Robot::handleAuxiliaryInput() {

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//	Buttons for climbing switches on the toggle button: left_trigger
	//
	//.................................................................................
	if (joystick_aux->GetRawButtonPressed(aux_buttons["left_bumper"])) {
		cout << "LEFT BUMPER" << endl;
		is_climbing_second = !is_climbing_second;
	}

	if (joystick_aux->GetRawButtonPressed(aux_buttons["right_bumper"])) {
		cout << "RIGHT BUMPER" << endl;
		is_climbing_third = !is_climbing_third;
	}
	

	if(joystick_aux->GetRawButton(aux_buttons["b"])) {
		elevator->setPosition(40);
	} else if(joystick_aux->GetRawButton(aux_buttons["y"])) {
		elevator->setPosition(75);
	} else if(joystick_aux->GetRawButton(aux_buttons["a"]) && elevator->getPosition() > 10) {
		elevator->setPosition(5);
	} else {
		elevator->setPower(0);
	}


	// if (joystick_aux->GetRawButton(aux_buttons["back"])) {
	// 		elevator->setPower(.15);
	// } else if (joystick_aux->GetRawButton(aux_buttons["start"])) {
	// 		elevator->setPower(-.45);
	// } else {
	// 	elevator->setPower(0);
	// }

	// if (joystick_aux->GetRawButton(aux_buttons["right_bumper"])) {
	// 	// start intake
	// 	cout << "RIGHT BUMPER" << endl;
	// 	intake->setIntakePower(intake_power);
	// } else if (joystick_aux->GetRawButton(aux_buttons["y"])) {
	// 	// start shooting
	// 	cout << "Y" << endl;
	// 	intake->setIntakePower(-intake_power);
	// } else {
	// 	intake->stop();
	// }

	if(is_climbing_second || is_climbing_third) {
		vector<float> values = gyro_correction->adjustClimb();

		// climb until we reach the max point, then stop lifting
		if(!is_climbing_complete && is_climbing_second) {
			if(climber->lowerClimberSecond(values[0], values[1])) {
				cout << "done climbing" << endl;
				is_climbing_complete = true;
			}
		}

		if(!is_climbing_complete && is_climbing_third) {
			if(climber->lowerClimberThird(values[0], values[1])) {
				cout << "done climbing" << endl;
				is_climbing_complete = true;
			}
		}

		// one we reach the top, sustain both sides until we specify not to with buttons
		if(is_climbing_complete) {

			// low_rider_timer.Start();

			// if(low_rider_timer.Get() < ) {
			// 	climber->setLowRider(-0.5);
			// }
			climber->setLowRider(joystick_aux->GetRawAxis(aux_axis["left_y"]));

			// TODO make sure raising is done
			
			
			// is_raising_back = true;
			// is_raising_front = true;

			// the front is no longer lowered, start raising it
			if(is_raising_front) {
				if(!is_front_raised) {
					if(climber->raiseFront()) {
						cout << "front finish raising" << endl;
						is_front_raised = true;
						is_raising_front = false;
						climber->stopFront();
					}
				}
			} else if(is_front_raised) {
				climber->stopFront();
			} else {
				climber->sustainFront();
			}

			// the back is no longer lowered, start raising it
			if(is_raising_back) {
				if(!is_back_raised) {
					if(climber->raiseBack()) {
						cout << "back finish raising" << endl;
						is_back_raised = true;
						is_raising_back = false;
						climber->stopBack();
					}
				}
			} else if(is_back_raised) {
				climber->setLowRider(-1);
				climber->stopBack();
			} else {
				climber->sustainBack();
			}


			if(joystick_aux->GetRawButtonPressed(aux_buttons["x"])) {
				cout << "X" << endl;
				is_raising_front = true;
			}

			if(joystick_aux->GetRawButtonPressed(aux_buttons["b"])) {
				cout << "B" << endl;
				is_raising_back = true;
			}
		}

	} else {
		// climber->setLiftPower(-joystick_aux->GetRawAxis(aux_axis["left_y"]), -joystick_aux->GetRawAxis(aux_axis["right_y"]));
	
		
		// climber->setLowRider(joystick_aux->GetRawAxis(aux_axis["left_x"]) / 2);
		// climber->stop();

	}
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
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

void Robot::AutonomousPeriodic() {

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

	// vector<float> f = gyro_correction->adjustClimb();

	// elevator->setPosition(75);
	// cout << joystick_aux->GetRawAxis(aux_axis["left_y"]) << endl;
	update();
}

void Robot::TeleopInit() {
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

void Robot::TeleopPeriodic() {
	update();
}

void Robot::update() {
	
	handleDriverInput();
	handleAuxiliaryInput();
	//elevator->getPosition();

	// vision_tracking->logLimelightValue("thor");
	if(is_climbing_second || is_climbing_third) {
		drive->TankDrive(abs(joystick_l->GetRawAxis(1)), abs(joystick_r->GetRawAxis(1)), false);
	} else if(setpoint_l != 0 || setpoint_r != 0 ) {
		// cout << "going to setpoint" << endl;
		drive->driveSetTicks(setpoint_l, setpoint_r);
	} else {
		drive->TankDrive(joystick_l->GetRawAxis(1) + adjust_l, joystick_r->GetRawAxis(1) + adjust_r, false);
		resetDriveValues();
	}
}

void Robot::DisabledInit() {
	cout << "I'm disabled :D" << endl;
}


void Robot::TestPeriodic() {
}

void Robot::resetDriveValues() {
	adjust_l = 0;
	adjust_r = 0;

	setpoint_l = 0.0;
	setpoint_r = 0.0;
}

void Robot::setSetpoints(float l, float r) {
	this->setpoint_l = l;
	this->setpoint_r = r;
}

void Robot::setAdjustments(float l, float r) {
	this->setpoint_l = l;
	this->setpoint_r = r;
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif

