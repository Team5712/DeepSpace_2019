#include "control_systems/Intake.h"

#include "control_systems/GMDrive.h"
#include "ctre/Phoenix.h"

#include <iostream>

using namespace std;

Intake::Intake() {
	cout << "intake initialized" << endl;
    intake = new WPI_VictorSPX(13);

    beak = new Solenoid(1);
}

void Intake::setIntakePower(float intake_power) {
    intake->Set(intake_power);
}

void Intake::intakeBall() {
    intake->Set(intake_power);
}

void Intake::releaseBall() {
    intake->Set(release_power);
}

void Intake::stopIntake() {
    intake->Set(0);
}

void Intake::updateBeak(bool update) {
    beak->Set(update);
}

void Intake::stop() {
    intake->Set(0);
}
