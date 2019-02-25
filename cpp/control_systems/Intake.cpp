#include "control_systems/Intake.h"

#include "control_systems/GMDrive.h"
#include "ctre/Phoenix.h"

#include <iostream>

using namespace std;

Intake::Intake() {
	cout << "intake initialized" << endl;
    intake = new WPI_VictorSPX(10);

}

void Intake::setIntakePower(float intake_power) {
    intake->Set(intake_power);
}

void Intake::stop() {
    intake->Set(0);
}

