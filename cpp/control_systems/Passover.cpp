#include "control_systems/Passover.h"

using namespace std;

Passover::Passover() {

    motor = new rev::SparkMax(9);

    pot = new AnalogPotentiometer(1, 100, 0); 

    piston = new Solenoid(2);   
}

float Passover::getPosition() {
    return pot->Get(); 
}

void Passover::printPotentiometer() {
    cout << "potentiometer " << pot->Get() << endl;
    // cout << "encoder " << motor->Get() << endl;
}

void Passover::setPosition(float setpoint) {

    // error is the distance from where we want to go from where we are now
    float error = setpoint - getPosition();

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
    float final_output = -fminf(fmaxf(output, pid.Kminoutput), pid.Kmaxoutput);
    // cout << "final output" << final_output << endl;
    // cout << "error " << error << endl;

    setPower(final_output);
}


// TODO: check directions practice, positive is backwards
void Passover::setPower(float power) {

    // if(max_front >) {

    // } 

    // // cout << "power " << power << endl;
    // cout << "pot " << getPosition() << endl;
    // motor->Set(power);
    // going to the front TODO: check these
    if(power > 0) {
        if(getPosition() >= max_back) {
            motor->Set(power);
        } else {
            motor->Set(0);
        }
        // going to the back
    } else if(power < 0) {
        if(getPosition() <= max_front) {
            motor->Set(power);
        } else {
            motor->Set(0);
        }
    } else {
        motor->Set(0);
    }  
    //COMP BOT
    // if(power < 0) {
    //     if(getPosition() <= max_front) {
    //         motor->Set(power);
    //     } else {
    //         motor->Set(0);
    //     }
    //     // going to the back
    // } else if(power > 0) {
    //     if(getPosition() >= max_back) {
    //         motor->Set(power);
    //     } else {
    //         motor->Set(0);
    //     }
    // } else {
    //     motor->Set(0);
    // }
}

bool Passover::isFront() {
    if(getPosition() > max_front - polar_range) {
        return true;
    } else {
        return false;
    }
}

bool Passover::isBack() {
    if(getPosition() < max_back + polar_range) {
        return true;
    } else {
        return false;
    }
}

void Passover::sendPassoverBack() {
    setPosition(setpoint_back);
}

void Passover::sendPassoverFront() {
    setPosition(setpoint_front);
}

void Passover::sendPassoverMiddle() {
    setPosition(setpoint_middle);
}

void Passover::sendPassoverFrontCargo() {
    setPosition(setpoint_front_cargo);
}

bool Passover::isPolarized() {
    if(getPosition() > max_front - 2 || getPosition() < max_back + 2) {
        return true;
    } else {
        return false;
    }
}

void Passover::setPiston(bool postition) {
    piston->Set(postition);
}