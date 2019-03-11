#ifndef SRC_CONTROL_SYSTEMS_GMDRIVE_H_
#define SRC_CONTROL_SYSTEMS_GMDRIVE_H_

#include <cmath>
#include <iostream>


#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

using namespace std;

/**
 * this class will implement many of the basic functionalities of our drive system
 * into one convenient class with drive and turning options
 * 
 */
class GMDrive
{
  public:
	GMDrive(int *, int *);
	~GMDrive();

	void SetSafetyEnabled(bool);
	void TankDrive(float, float, bool);

	void driveSetTicks(float, float);
	void driveSetInches(float, float);
	void driveSetFeet(float, float);


	void ConfigMotionCruiseVelocityLeft(float);
	void ConfigMotionCruiseVelocityRight(float);

	void ConfigMotionAccelerationLeft(float);
	void ConfigMotionAccelerationRight(float);

	// will set the cruise velocity to the original velocity minus the given value
	void ConfigMotionCruiseVelocityDifferenceLeft(float);
	void ConfigMotionCruiseVelocityDifferenceRight(float);

	void resetEncoders();

	float getRightTicks();
	float getLeftTicks();

	void updateShifter(bool);

	void initPID();

  private:
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Ticks per revolution / circumference of the wheel (inches)
	//
	// This is basically number of ticks per revolution per inch
	// multiply this number by the number of inches you want to go
	// to get your desired amout of ticks
	//.......................................................................................
	const float ratio = (515 / (2 * 2 * M_PI));

	WPI_TalonSRX *l_master;
	WPI_TalonSRX *r_master;

	rev::CANSparkMax *l_slave1;
	rev::CANSparkMax *l_slave2;
	rev::CANSparkMax *r_slave1;
	rev::CANSparkMax *r_slave2;

	frc::Solenoid *shifter;

	// use for programming chassis and 2018 bot
	// WPI_VictorSPX *l_slave1;
	// WPI_VictorSPX *l_slave2;
	// WPI_VictorSPX *l_slave3;

	// WPI_VictorSPX *r_slave1;
	// WPI_VictorSPX *r_slave2;
	// WPI_VictorSPX *r_slave3;

	frc::DifferentialDrive *drive;

	struct pid_values
	{
		const float wheel_radius = 3.0;
		int ticks_per_revolution = 128;
		// old 168.01
		const float RATIO = ((ticks_per_revolution / (2 * wheel_radius * M_PI)));
		// 6.7061664
		// 18.84954

		// speed in ticks/100ms
		float max_speed = 440;
		double rpm = 461;

		float l_scale = 1.0;
		float r_scale = 1.0;

		float gyro_min = 4.0;
		float gyro_Kp = 9.0;
		int gyro_skip = 2;

		// ticks / 100 ms / second
		float accel = this->max_speed * 1.0;
		// maximum ticks / 100 ms
		float vel = this->max_speed * 1.0;
		// % * constant / error
		// (1.00 * 128) / 250 = 0.512
		const float Kp = 1.0;
		const float Ki = 0.0;
		const float Kz = 0.0;
		const float Kd = 0.0;
		//
		const float Kf = (0.0);
		const float timeout = 0.005;
	} pid;
};

#endif /* SRC_CONTROL_SYSTEMS_GMDRIVE_H_ */
