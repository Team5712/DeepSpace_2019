// #ifndef SRC_INPUT_H_
// #define SRC_INPUT_H_

// #include <iostream>
// #include <cmath>
// #include <map>
// #include <string>
// #include <vector>

// #include "frc/WPILib.h"

// #include "vision/Tracking.h"

// using namespace std;

// class Input {
// public:
// 	Input();
//     // Input(float setpoint_l, float setpoint_r, float adjust_l, float adjust_r, float joystick_l, float joystick_r);

//     void handleJoystickInput();
//     void handleAuxInput();

// private:

//     frc::Joystick *joystick_l;
//     frc::Joystick *joystick_r;
//     frc::Joystick *joystick_aux;

//     Tracking *vision_tracking;

//     map<string, int> driver_buttons = {
//         {"trigger", 1},
//         {"left", 2},
//         {"right", 3},
//         {"middle", 4},
//     };

//     // input mapping for aux controller
//     map<string, int> aux_buttons = {
//         {"a", 1},
//         {"b", 2},
//         {"x", 3},
//         {"y", 4},
//         {"left_bumper", 5},
//         {"right_bumper", 6},
//         {"back", 7},
//         {"start", 8},
//         {"left_click", 9},
//         {"right_click", 10},
//     };


//     map<string, int> driver_axis = {
//         {"trigger", 1},
//         {"left", 2},
//         {"right", 3},
//         {"middle", 4},
//     };

//     map<string, int> aux_axis = {
//         {"trigger", 1},
//         {"left", 2},
//         {"right", 3},
//         {"middle", 4},
//     };


// 	virtual ~Input();
// };

// #endif /* SRC_INPUT_H_ */
