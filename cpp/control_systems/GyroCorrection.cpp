#include "control_systems/GyroCorrection.h"

GyroCorrection::GyroCorrection(AHRS *gyro) {
    this->gyro = gyro;
}

vector<float> GyroCorrection::adjustDrive() {
    if(gyro_skip <= 0) {
		gyro_error = abs(gyro->GetYaw());
        // cout << gyro->GetYaw() << endl;
    }

	// right
	if(gyro->GetYaw() > 0) {
		// cout << "curving right - correcting to the left: " << gyro_correction << endl;
		l_gyro_adjust = gyro_Kp * gyro_error + gyro_min;

	// left
	} else if(gyro->GetYaw() < 0) {
		// cout << "curving left - correcting to the right: " << gyro_correction << endl;
		r_gyro_adjust = gyro_Kp * gyro_error + gyro_min;
	}

	gyro_skip -= 1;
	
    vector<float> values = {l_gyro_adjust, r_gyro_adjust};
    return values;
	// drive->ConfigMotionCruiseVelocityLeft(pid.vel - gyro_correction.l_gyro_adjust);
	// drive->ConfigMotionCruiseVelocityRight(pid.vel - gyro_correction.r_gyro_adjust);
}

vector<float> GyroCorrection::adjustClimb() {
	
	pitch_error = abs(gyro->GetRoll());

	float back_pitch_adjust;
	float front_pitch_adjust;
	// cout << "pitch: " << gyro->GetRoll() - pitch_zero << "zer " << pitch_zero << endl;
	// back
	if(gyro->GetRoll() - pitch_zero > -3) {
		 cout << "tilting back - raise back legs: " << gyro_correction << endl;
		back_pitch_adjust = pitch_Kp * pitch_error;
	// front
	} else if(gyro->GetRoll() - pitch_zero < -7) {
		 cout << "tilting forward  - raise front legs: " << gyro_correction << endl;
		front_pitch_adjust = pitch_Kp * pitch_error;
	} else {
		
	}
	// cout << "front " << front_pitch_adjust << " back " << back_pitch_adjust << endl;
	vector<float> values = {front_pitch_adjust, back_pitch_adjust};
	return values; 
}

void GyroCorrection::zeroPitch() {
	cout << gyro->GetRoll() << endl;
	cout << gyro->GetRoll() << endl;
	pitch_zero = gyro->GetRoll();
}