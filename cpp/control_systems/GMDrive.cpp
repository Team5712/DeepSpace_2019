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

	//l_master = new WPI_TalonSRX(left[0]);
	//r_master = new WPI_TalonSRX(right[0]);
	l_master = new rev::CANSparkMax(left[0], rev::CANSparkMax::MotorType::kBrushless);
	r_master = new rev::CANSparkMax(right[0], rev::CANSparkMax::MotorType::kBrushless);

	l_slave1 = new rev::CANSparkMax(left[1], rev::CANSparkMax::MotorType::kBrushless);
	l_slave2 = new rev::CANSparkMax(left[2], rev::CANSparkMax::MotorType::kBrushless);
	r_slave1 = new rev::CANSparkMax(right[1], rev::CANSparkMax::MotorType::kBrushless);
	r_slave2 = new rev::CANSparkMax(right[2], rev::CANSparkMax::MotorType::kBrushless);

	l_slave1->Follow(*l_master);
	l_slave2->Follow(*l_master);

	r_slave1->Follow(*r_master);
	r_slave2->Follow(*r_master);

	// l_master->SetClosedLoopRampRate(0.3);
	// r_master->SetClosedLoopRampRate(0.3);

	// l_slave1->SetClosedLoopRampRate(0.3);
	// l_slave2->SetClosedLoopRampRate(0.3);

	// r_slave1->SetClosedLoopRampRate(0.3);
	// r_slave2->SetClosedLoopRampRate(0.3);

	//rev::CANSparkMax::ExternalFollower left_master_follower(rev::CANSparkMax::kFollowerPhoenix);
	//l_slave1->Follow(left_master_follower, 1, false);
	//l_slave2->Follow(left_master_follower, 1, false);

	// rev::CANSparkMax::ExternalFollower right_master_follower(rev::CANSparkMax::kFollowerPhoenix);
	// r_slave1->Follow(right_master_follower, 4, false);
	// r_slave2->Follow(right_master_follower, 4, false);


	shifter = new frc::Solenoid(0); //PRACTICE is 3 //COMP is 0
	
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
}

void GMDrive::driveSetFeet(float setpoint_l, float setpoint_r) {
	//std::cout << "left " << l_master->GetSelectedSensorPosition(0) << endl;
	//std::cout << "right " << r_master->GetSelectedSensorPosition(0) << endl;
	//l_master->Set(ControlMode::MotionMagic, setpoint_l * ratio * 12);
	//r_master->Set(ControlMode::MotionMagic, -setpoint_r * ratio * 12);
}

void GMDrive::driveSetTicks(float setpoint_l, float setpoint_r) {
	//l_master->Set(ControlMode::MotionMagic, setpoint_l);
	//r_master->Set(ControlMode::MotionMagic, -setpoint_r);
}


// void GMDrive::setBackLiftPower(float power) {
// 	//back_lift->Set(power);
// 	cout << "right " << power << endl;
// }

void GMDrive::driveSetInches(float setpoint_l, float setpoint_r) {
	//std::cout << "left value: " << l_master->GetSelectedSensorPosition(0) << " err " << l_master->GetClosedLoopError(0) << " d: " << l_master->GetSelectedSensorPosition(0) - setpoint_l << endl;
   // std::cout << "right value: " << r_master->GetSelectedSensorPosition(0) << " err " << r_master->GetClosedLoopError(0) << " d: " << l_master->GetSelectedSensorPosition(0) - setpoint_r << endl;
	//l_master->Set(ControlMode::MotionMagic, setpoint_l * ratio);
	//r_master->Set(ControlMode::MotionMagic, -setpoint_r * ratio);
}


bool GMDrive::setPositionTicks(float left_setpoint, float right_setpoint)
{


	float left_error = left_setpoint + getLeftTicks();
	float right_error = right_setpoint - getRightTicks();

	if(abs(left_error) < margin_distance && abs(right_error) < margin_distance) {
		this->TankDrive(0, 0, false);
		return true;
	}

	// error is the distance from where we want to go from where we are now
	//float left_error = left_setpoint - l_master->GetSelectedSensorPosition(0);
	//float right_error = right_setpoint - r_master->GetSelectedSensorPosition(0);
	//cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;

	// calculate proportion value
	float left_p = pid.Kp * left_error;
	float right_p = pid.Kp * right_error;

	// i_zone for perfecting distance to target
	if (fabsf(left_error) <= pid.i_zone || pid.i_zone == 0.0f)
	{
		pid.left_i_state = pid.left_i_state + (left_error * pid.Ki);
	}
	else
	{
		pid.left_i_state = 0;
	}

	// i_zone for perfecting distance to target
	if (fabsf(right_error) <= pid.i_zone || pid.i_zone == 0.0f)
	{
		pid.right_i_state = pid.right_i_state + (right_error * pid.Ki);
	}
	else
	{
		pid.right_i_state = 0;
	}


	float left_d = (left_error - pid.left_prev_error);
	pid.left_prev_error = left_error;
	left_d *= pid.Kd;

	float right_d = (right_error - pid.right_prev_error);
	pid.right_prev_error = right_error;
	right_d *= pid.Kd;

	// static feed forward value based on how far we need to go
	float left_f = left_setpoint * pid.Kf;
	float right_f = right_setpoint * pid.Kf;

	// add up all of our values for our output
	float left_output = left_p + pid.left_i_state + left_d + left_f;
	float right_output = right_p + pid.right_i_state + right_d + right_f;

	// make sure the output is not greater than our max or less than our min
	float left_final_output = fminf(fmaxf(left_output, pid.min_output), pid.max_output);
	float right_final_output = fminf(fmaxf(right_output, pid.min_output), pid.max_output);
	// cout << "output left " << left_final_output << "output right " << right_final_output << endl;
	// cout << "err left " << left_error << " right " << right_error << endl;
	this->TankDrive(-left_final_output, -right_final_output, false);

	return false;
}


void GMDrive::setPositionInches(float left_position, float right_position) {
	setPositionTicks(left_position * ratio, right_position * ratio);
}

void GMDrive::initPID() {
	// l_master->SetSensorPhase(false);
	// r_master->SetSensorPhase(true);

	// l_master->ConfigPeakOutputForward(12, 0.001);
	// l_master->ConfigPeakOutputReverse(-12, 0.001);

	// r_master->ConfigPeakOutputForward(12, 0.001);
	// r_master->ConfigPeakOutputReverse(-12, 0.001);

	// l_master->ConfigNominalOutputForward(0, 0.001);
	// l_master->ConfigNominalOutputReverse(-0, 0.001);

	// r_master->ConfigNominalOutputForward(0, 0.001);
	// r_master->ConfigNominalOutputReverse(-0, 0.001);

	// l_master->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
	// 		pid.timeout);
	// r_master->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
	// 		pid.timeout);

	// l_master->SelectProfileSlot(0, 0);
	// l_master->Config_kP(0, pid.Kp, pid.timeout);
	// l_master->Config_kI(0, pid.Ki, pid.timeout);
	// l_master->Config_kD(0, pid.Kd, pid.timeout);
	// l_master->Config_kF(0, pid.Kf, pid.timeout);
	// l_master->Config_IntegralZone(0, pid.Kz, pid.timeout);

	// r_master->SelectProfileSlot(0, 0);
	// r_master->Config_kP(0, pid.Kp, pid.timeout);
	// r_master->Config_kI(0, pid.Ki, pid.timeout);
	// r_master->Config_kD(0, pid.Kd, pid.timeout);
	// r_master->Config_kF(0, pid.Kf, pid.timeout);
	// r_master->Config_IntegralZone(0, pid.Kz, pid.timeout);

	// //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// // Note: these values are are measured to be the fastest speed our robot can attain units/100ms
	// // These may be scaled down so that they are more consistant with one another
	// //..............................................................................................
	// l_master->ConfigMotionAcceleration(pid.accel, pid.timeout);
	// l_master->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);

	// r_master->ConfigMotionAcceleration(pid.accel, pid.timeout);
	// r_master->ConfigMotionCruiseVelocity(pid.vel, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityLeft(float value) {
	//l_master->ConfigMotionCruiseVelocity(value, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityRight(float value) {
	//r_master->ConfigMotionCruiseVelocity(value, pid.timeout);
}

void GMDrive::ConfigMotionAccelerationLeft(float value) {
	//l_master->ConfigMotionAcceleration(value, pid.timeout);
}

void GMDrive::ConfigMotionAccelerationRight(float value) {
	//r_master->ConfigMotionAcceleration(value, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityDifferenceLeft(float value) {
	//l_master->ConfigMotionCruiseVelocity(pid.vel - value, pid.timeout);
}

void GMDrive::ConfigMotionCruiseVelocityDifferenceRight(float value) {
	//r_master->ConfigMotionCruiseVelocity(pid.vel - value, pid.timeout);
}

float GMDrive::getLeftTicks() {
	// TODO: CHECK THIS
	return l_master->GetEncoder().GetPosition();
}

float GMDrive::getRightTicks() {
	return r_master->GetEncoder().GetPosition(); 
}

bool GMDrive::turnLeft() {
	TankDrive(-0.3, 0.3, false);
	
}

bool GMDrive::turnRight() {
	
}


void GMDrive::updateShifter(bool update) {
	shifter->Set(update);
}

void GMDrive::resetEncoders() {
	l_master->GetEncoder().SetPosition(0);
	r_master->GetEncoder().SetPosition(0);
}

GMDrive::~GMDrive() {
}

