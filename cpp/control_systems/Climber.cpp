#include "control_systems/Climber.h"

#include "control_systems/GMDrive.h"
#include "ctre/Phoenix.h"

Climber::Climber()
{
	cout << "climber initialized" << endl;
	
	back_lift = new WPI_TalonSRX(7);
	front_lift = new WPI_TalonSRX(8);
	low_rider = new WPI_VictorSPX(9);

	front_lift->SetSensorPhase(false);
	back_lift->SetSensorPhase(false);
}

void Climber::setLiftPower(float front, float back)
{
	//cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;
	//cout << "back " << back_lift->GetSelectedSensorPosition(0) << endl;
	front_lift->Set(front);
	back_lift->Set(-back);
}

// negative is down positive is up
bool Climber::lowerClimberThird(float front, float back)
{
	// cout << "front " << front << " back" << back << endl;
	// cout << "back3 " << abs(back_lift->GetSelectedSensorPosition(0)) << " front " << abs(front_lift->GetSelectedSensorPosition(0)) << endl;

	if ((abs(front_lift->GetSelectedSensorPosition(0)) <= climb_third) || (abs(back_lift->GetSelectedSensorPosition(0)) <= climb_third))
	{
		front_lift->Set(climb_speed + front_offset - front);
		// TODO: add this back in
		back_lift->Set(-climb_speed - back);
		return false;
	}
	else
	{
		return true;
	}
}

// negative is down positive is up
bool Climber::lowerClimberSecond(float front, float back)
{
	// cout << "front " << front << " back" << back << endl;
	// cout << "back2 " << abs(back_lift->GetSelectedSensorPosition(0)) << " front " << abs(front_lift->GetSelectedSensorPosition(0)) << endl;

	if ((abs(front_lift->GetSelectedSensorPosition(0)) <= climb_second) || (abs(back_lift->GetSelectedSensorPosition(0)) <= climb_second))
	{
		front_lift->Set(climb_speed + front_offset - front);
		// TODO: add this back in
		back_lift->Set(-climb_speed - back);
		return false;
	}
	else
	{
		return true;
	}
}

void Climber::sustainLift()
{
	front_lift->Set(-climb_sustain);
	back_lift->Set(climb_sustain);
}

void Climber::sustainBack()
{
	back_lift->Set(climb_sustain);
}

void Climber::sustainFront()
{
	front_lift->Set(-climb_sustain);
}

bool Climber::raiseFront()
{
	// cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;
	if (front_lift->GetSelectedSensorPosition(0) >= climb_max)
	{
		front_lift->Set(-raise_speed);
		return false;
	}
	else
	{
		cout << "DONE FRONT"  << endl;
		return true;
	}
}

bool Climber::raiseBack()
{
	// cout << "back " << back_lift->GetSelectedSensorPosition(0) << endl;
	if (back_lift->GetSelectedSensorPosition(0) >= climb_max)
	{
		back_lift->Set(raise_speed);
		return false;
	}
	else
	{
		cout << "DONE BACK"  << endl;
		return true;
	}
}

void Climber::setPositions(float front_setpoint, float back_setpoint)
{

	// error is the distance from where we want to go from where we are now
	float front_error = front_setpoint - front_lift->GetSelectedSensorPosition(0);
	float back_error = back_setpoint - back_lift->GetSelectedSensorPosition(0);
	//cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;

	// calculate proportion value
	float front_p = front_pid.Kp * front_error;
	float back_p = back_pid.Kp * back_error;

	// i_zone for perfecting distance to target
	if (fabsf(front_error) <= front_pid.i_zone || front_pid.i_zone == 0.0f)
	{
		front_pid.i_state = front_pid.i_state + (front_error * front_pid.Ki);
	}
	else
	{
		front_pid.i_state = 0;
	}

	// i_zone for perfecting distance to target
	if (fabsf(back_error) <= back_pid.i_zone || back_pid.i_zone == 0.0f)
	{
		back_pid.i_state = back_pid.i_state + (back_error * back_pid.Ki);
	}
	else
	{
		back_pid.i_state = 0;
	}

	//
	float front_d = (front_error - front_pid.prev_error);
	front_pid.prev_error = front_error;
	front_d *= front_pid.Kd;

	float back_d = (back_error - back_pid.prev_error);
	back_pid.prev_error = back_error;
	back_d *= back_pid.Kd;

	// static feed forward value based on how far we need to go
	float front_f = front_setpoint * front_pid.Kf;
	float back_f = back_setpoint * back_pid.Kf;

	// add up all of our values for our output
	float front_output = front_p + front_pid.i_state + front_d + front_f;
	float back_output = back_p + back_pid.i_state + back_d + back_f;

	// make sure the output is not greater than our max or less than our min
	float front_final_output = -fminf(fmaxf(front_output, front_pid.Kminoutput), front_pid.Kmaxoutput);
	float back_final_output = -fminf(fmaxf(back_output, back_pid.Kminoutput), back_pid.Kmaxoutput);
	cout << "front final output" << front_final_output << endl;

	setLiftPower(front_final_output, back_final_output);
}

void Climber::stop()
{
	back_lift->Set(0);
	front_lift->Set(0);
}

void Climber::stopFront()
{
	front_lift->Set(0);
}

void Climber::stopBack()
{
	back_lift->Set(0);
}

void Climber::resetEncoders()
{
	front_lift->SetSelectedSensorPosition(0, 0, 50);
	back_lift->SetSelectedSensorPosition(0, 0, 50);
}

void Climber::setLowRider(float power)
{
	low_rider->Set(power);
}

void Climber::printEncoders() {
	cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;
	cout << "back " << back_lift->GetSelectedSensorPosition(0) << endl;
}