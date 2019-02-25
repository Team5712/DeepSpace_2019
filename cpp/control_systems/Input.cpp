

// #include "control_systems/Input.h"

// Input::Input() {
// 	joystick_l = new frc::Joystick(0);
// 	joystick_r = new frc::Joystick(1);
// 	joystick_aux = new frc::Joystick(2);

//     vision_tracking = new Tracking();
// }

// void Input::handleJoystickInput() {
//     // 	// track vision tape
// 	// if(joystick_l->GetRawButton(driver_buttons["middle"])) {
// 	// 	cout << "left middle" << endl;

// 	// 	vector<float> adjust = vision_tracking->getTurnAdjustmentPercents(0);
        


// 	// // track cargo
// 	// } else if(joystick_r->GetRawButton(driver_buttons["middle"])) {
// 	// 	cout << "right middle" << endl;

// 	// 	vector<float> adjust = vision_tracking->getTurnAdjustmentPercents(1);


// 	// 	// TODO: wait based on variable in tracking.hpp instead of doing it all the time
// 	// 	// drive to setpoint from given target
// 	// } else if(joystick_l->GetRawButtonPressed(driver_buttons["trigger"])) {
// 	// 	cout << "left trigger" << endl;

// 	// 	vector<float> setpoint = vision_tracking->getTurnAdjustmentTicks(0);


// 	// } else if(joystick_l->GetRawButton(driver_buttons["trigger"])) {
// 	// 	if(abs(drive->getLeftTicks() - setpoint_l) < 40 || abs(drive->getRightTicks() - setpoint_r) < 40
// 	// 			&& (setpoint_l <= 0 || setpoint_r <= 0)) {
// 	// 		cout << "done turning" << endl;
// 	// 		drive->resetEncoders();
// 	// 		//cout << "l " << abs(drive->getLeftTicks() - setpoint_l) << "r " << abs(drive->getRightTicks() - setpoint_r) << endl;
// 	// 		float setpoint = vision_tracking->getDriveFromSetPointTicks(12);
// 	// 		setpoint_l = setpoint;
// 	// 		setpoint_r = setpoint;
// 	// 		cout << "DISTANCE SET " << setpoint << endl;
// 	// 	}

// 	// 	if((abs(drive->getLeftTicks() - setpoint_l) < 60 || abs(drive->getRightTicks() - setpoint_r) < 60) && (setpoint_l > 0 && setpoint_r > 0)) {
// 	// 		cout << "done driveing " << setpoint_l << " " << setpoint_r << endl;
// 	// 		setpoint_l = 0;
// 	// 		setpoint_r = 0;
// 	// 	}
// 	// } else if(joystick_l->GetRawButtonReleased(driver_buttons["trigger"])) {
// 	// 	cout << "stop" << endl;
// 	// 	resetDriveValues();
// 	// } else if (!joystick_l->GetRawButton(driver_buttons["trigger"])) {
// 	// 	drive->resetEncoders();
// 	// }
// }


// void Input::handleAuxInput() {

// }
