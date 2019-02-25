#include "control_systems/Climber.h"

#include "control_systems/GMDrive.h"
#include "ctre/Phoenix.h"


Climber::Climber() {
	cout << "climber initialized" << endl;
    back_lift = new WPI_TalonSRX(7);
	front_lift = new WPI_TalonSRX(8);
	low_rider = new WPI_VictorSPX(9);

	front_lift->SetSensorPhase(false);
	back_lift->SetSensorPhase(false);
}

void Climber::setLiftPower(float front, float back) {
	cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;
	cout << "back " << back_lift->GetSelectedSensorPosition(0) << endl;
	front_lift->Set(front);
	back_lift->Set(-back);
}

// negative is down positive is up
bool Climber::lowerClimberThird(float front, float back) {
	// cout << "front " << front << " back" << back << endl;
	cout << "back3 " << abs(back_lift->GetSelectedSensorPosition(0)) << " front " << abs(front_lift->GetSelectedSensorPosition(0)) << endl;
	
	if((abs(front_lift->GetSelectedSensorPosition(0)) <= climb_third) || (abs(back_lift->GetSelectedSensorPosition(0)) <= climb_third)) {
		front_lift->Set(climb_speed - front);
		// TODO: add this back in
		back_lift->Set(-climb_speed - back);
		return false;
	} else {
		return true;
	}
}


// negative is down positive is up
bool Climber::lowerClimberSecond(float front, float back) {
	// cout << "front " << front << " back" << back << endl;
	cout << "back2 " << abs(back_lift->GetSelectedSensorPosition(0)) << " front " << abs(front_lift->GetSelectedSensorPosition(0)) << endl;
	
	if((abs(front_lift->GetSelectedSensorPosition(0)) <= climb_second) || (abs(back_lift->GetSelectedSensorPosition(0)) <= climb_second)) {
		front_lift->Set(climb_speed - front);
		// TODO: add this back in
		back_lift->Set(-climb_speed - back);
		return false;
	} else {
		return true;
	}
}


void Climber::sustainLift() {
	front_lift->Set(-climb_sustain);
	back_lift->Set(climb_sustain);
}

void Climber::sustainBack() {
	back_lift->Set(climb_sustain);
}

void Climber::sustainFront() {
	front_lift->Set(-climb_sustain);
}

bool Climber::raiseFront() {
	 cout << "front " << front_lift->GetSelectedSensorPosition(0) << endl;
	if(front_lift->GetSelectedSensorPosition(0) >= climb_max) {
		front_lift->Set(-raise_speed);
		return false;
	} else {
		return true;
	}
}

bool Climber::raiseBack() {
	cout << "back " << back_lift->GetSelectedSensorPosition(0) << endl;
	if(back_lift->GetSelectedSensorPosition(0) >= climb_max) {
		back_lift->Set(raise_speed);
		return false;
	} else {
		return true;
	}
}

void Climber::stop() {
	back_lift->Set(0);
	front_lift->Set(0);
}

void Climber::stopFront() {
	front_lift->Set(0);
}

void Climber::stopBack() {
	back_lift->Set(0);
}

void Climber::resetEncoders() {
	front_lift->SetSelectedSensorPosition(0, 0, 0.0001);
	back_lift->SetSelectedSensorPosition(0, 0, 0.0001);
}

void Climber::setLowRider(float power) {
	low_rider->Set(power);
}

