/*
 * Drive.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: robotics
 */

#include "control_systems/GMDrive.h"



GMDrive::GMDrive(int* left, int* right) {

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// initialize motor controllers
	//....................................................................

	l_master = new WPI_TalonSRX(left[0]);
	r_master = new WPI_TalonSRX(right[0]);
	
	l_slave1 = new rev::CANSparkMax(left[1], rev::CANSparkMax::MotorType::kBrushless);
	l_slave2 = new rev::CANSparkMax(left[2], rev::CANSparkMax::MotorType::kBrushless);
	r_slave1 = new rev::CANSparkMax(right[1], rev::CANSparkMax::MotorType::kBrushless);
	r_slave2 = new rev::CANSparkMax(right[2], rev::CANSparkMax::MotorType::kBrushless);

	rev::CANSparkMax::ExternalFollower left_master_follower(rev::CANSparkMax::kFollowerPhoenix);
	l_slave1->Follow(left_master_follower, 1, false);
	l_slave2->Follow(left_master_follower, 1, false);

	rev::CANSparkMax::ExternalFollower right_master_follower(rev::CANSparkMax::kFollowerPhoenix);
	r_slave1->Follow(right_master_follower, 4, false);
	r_slave2->Follow(right_master_follower, 4, false);
	
	//use following lines for programming chassis and 2018 bot
	// l_slave1 = new WPI_VictorSPX(left[1]);
	// l_slave2 = new WPI_VictorSPX(left[2]);
	// l_slave3 = new WPI_VictorSPX(left[3]);

	// r_slave1 = new WPI_VictorSPX(right[1]);
	// r_slave2 = new WPI_VictorSPX(right[2]);
	// r_slave3 = new WPI_VictorSPX(right[3]);

	// l_slave1->Follow(*l_master);
	// r_slave1->Follow(*r_master);

	this->drive = new frc::DifferentialDrive(*l_master, *r_master);
}

void GMDrive::SetSafetyEnabled(bool value) {
	this->drive->SetSafetyEnabled(value);
}

void GMDrive::TankDrive(float l, float r, bool squared_inputs) {
	this->drive->TankDrive(l, r, squared_inputs);
	// cout << "inches: " << this->getLeftTicks() / ratio << endl;
}

void GMDrive::driveSetFeet(float setpoint_l, float setpoint_r) {
	cout << "left " << l_master->GetSelectedSensorPosition(0) << endl;
	cout << "right " << r_master->GetSelectedSensorPosition(0) << endl;
	l_master->Set(ControlMode::MotionMagic, setpoint_l * ratio * 12);
	r_master->Set(ControlMode::MotionMagic, -setpoint_r * ratio * 12);
}

void GMDrive::driveSetTicks(float setpoint_l, float setpoint_r) {
	l_master->Set(ControlMode::MotionMagic, setpoint_l);
	r_master->Set(ControlMode::MotionMagic, -setpoint_r);
}


// void GMDrive::setBackLiftPower(float power) {
// 	//back_lift->Set(power);
// 	cout << "right " << power << endl;
// }

void GMDrive::driveSetInches(float setpoint_l, float setpoint_r) {
	cout << "left value: " << l_master->GetSelectedSensorPosition(0) << " err " << l_master->GetClosedLoopError(0) << " d: " << l_master->GetSelectedSensorPosition(0) - setpoint_l << endl;
    cout << "right value: " << r_master->GetSelectedSensorPosition(0) << " err " << r_master->GetClosedLoopError(0) << " d: " << l_master->GetSelectedSensorPosition(0) - setpoint_r << endl;
	l_master->Set(ControlMode::MotionMagic, setpoint_l * ratio);
	r_master->Set(ControlMode::MotionMagic, -setpoint_r * ratio);
}

void GMDrive::initPID() {
	l_master->SetSensorPhase(false);
	r_master->SetSensorPhase(true);

	l_master->ConfigPeakOutputForward(12, 0.001);
	l_master->ConfigPeakOutputReverse(-12, 0.001);

	r_master->ConfigPeakOutputForward(12, 0.001);
	r_master->ConfigPeakOutputReverse(-12, 0.001);

	l_master->ConfigNominalOutputForward(0, 0.001);
	l_master->ConfigNominalOutputReverse(-0, 0.001);

	r_master->ConfigNominalOutputForward(0, 0.001);
	r_master->ConfigNominalOutputReverse(-0, 0.001);

	l_master->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
			pid.timeout);
	r_master->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
			pid.timeout);

	l_master->SelectProfileSlot(0, 0);
	l_master->Config_kP(0, pid.Kp, pid.timeout);
	l_master->Config_kI(0, pid.Ki, pid.timeout);
	l_master->Config_kD(0, pid.Kd, pid.timeout);
	l_master->Config_kF(0, pid.Kf, pid.timeout);
	l_master->Config_IntegralZone(0, pid.Kz, pid.timeout);

	r_master->SelectProfileSlot(0, 0);
	r_master->Config_kP(0, pid.Kp, pid.timeout);
	r_master->Config_kI(0, pid.Ki, pid.timeout);
	r_master->Config_kD(0, pid.Kd, pid.timeout);
	r_master->Config_kF(0, pid.Kf, pid.timeout);
	r_master->Config_IntegralZone(0, pid.Kz, pid.timeout);

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Note: these values are are measured to be the fastest speed our robot can attain units/100ms
	// These may be scaled down so that they are more consistant with one another
	//..............................................................................................
	l_master->ConfigMotionAcceleration(pid.accel, pid.timeout);
	l_master->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);

	r_master->ConfigMotionAcceleration(pid.accel, pid.timeout);
	r_master->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityLeft(float value) {
	l_master->ConfigMotionCruiseVelocity(value, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityRight(float value) {
	r_master->ConfigMotionCruiseVelocity(value, pid.timeout);
}

void GMDrive::ConfigMotionAccelerationLeft(float value) {
	l_master->ConfigMotionAcceleration(value, pid.timeout);
}

void GMDrive::ConfigMotionAccelerationRight(float value) {
	r_master->ConfigMotionAcceleration(value, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityDifferenceLeft(float value) {
	l_master->ConfigMotionCruiseVelocity(pid.vel - value, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityDifferenceRight(float value) {
	r_master->ConfigMotionCruiseVelocity(pid.vel - value, pid.timeout);
}

float GMDrive::getLeftTicks() {
	return l_master->GetSelectedSensorPosition(0);
}

float GMDrive::getRightTicks() {
	return r_master->GetSelectedSensorPosition(0);
}

void GMDrive::resetEncoders() {
	l_master->SetSelectedSensorPosition(0, 0, pid.timeout);
	r_master->SetSelectedSensorPosition(0, 0, pid.timeout);
}

GMDrive::~GMDrive() {
}

