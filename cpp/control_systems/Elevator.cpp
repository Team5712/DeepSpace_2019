#include "control_systems/Elevator.h"

#include "control_systems/GMDrive.h"
#include "ctre/Phoenix.h"

#include <iostream>

using namespace std;

Elevator::Elevator() {

    elevator_slave = new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);
    elevator_master = new rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);

    // rev::CANSparkMax::ExternalFollower elevator_master_follower(rev::CANSparkMax::kFollowerSparkMax);
	// elevator_slave->Follow(elevator_master_follower, 12, false);

    // rev::CANSparkMax elevator_master{11, rev::CANSparkMax::MotorType::kBrushless};
    // rev::CANSparkMax elevator_slave{12, rev::CANSparkMax::MotorType::kBrushless};
    elevator_slave->Follow(*elevator_master);

    pot = new AnalogPotentiometer(0, 100, 0); 
    
}

float Elevator::getPosition() {
    return pot->Get(); 
}

void Elevator::setPosition(float setpoint) {

    // error is the distance from where we want to go from where we are now
    float error = setpoint - pot->Get();
    // cout << "pot " << pot->Get() << endl;

    // calculate proportion value
    float p = pid.Kp * error;

    // i_zone for perfecting distance to target
    if(fabsf(error) <= pid.i_zone || pid.i_zone == 0.0f) {
        pid.i_state = pid.i_state + (error * pid.Ki);
    } else {
        pid.i_state = 0;
    }

    // 
    float d = (error - pid.prev_error);
    pid.prev_error = error;
    d *= pid.Kd;

    // static feed forward value based on how far we need to go
    float f = setpoint * pid.Kf;

    // add up all of our values for our output
    float output = p + pid.i_state + d + f;

    // make sure the output is not greater than our max or less than our min
    float final_output = fminf(fmaxf(output, pid.Kminoutput), pid.Kmaxoutput);
    // cout << "final output" << final_output << endl;

    // TODO: COMP BOT IS POSITIVE, PRAC IS NEGATIVE
    setPower(final_output);
}


void Elevator::setPower(float power) {
    // cout << "power " << power << " pot " << pot->Get() << endl;
    if(pot->Get() < 10.5 && power < 0) {
        elevator_master->Set(0);
    } else { 
        elevator_master->Set(power);
    }
}