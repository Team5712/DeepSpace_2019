#ifndef SRC_VISION_H_
#define SRC_VISION_H_

#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "WPILib.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

using namespace std;

class Tracking {
public:
	Tracking();

	void setPipeline(int);

	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// 0. vision tape
	// 1. cargo
	//.........................................................

	vector<map<char, float>> Kw;

	/**
	 *
	 * calculate the adjustment values based on the given pipeline
	 *
	 * @param {int} limelight pipeline for the object we're tracking
	 * @returns vector<float>
	 *
	 */
	vector<float> getTurnAdjustmentPercents(int, int);


	/**
	 * 
	 * @param {int} limelight pipeline
	 */
	vector<float> getTurnAdjustmentTicks(int, int);

	/**
	 * calculate the set point for the currently tracked vision object
	 *
	 * @param {int} pipeline for the object we're tracking
	 * @param {float} distance you would like to be from the target (inches)
	 */
	float getDriveFromSetPointTicks(float);

	void logLimelightValue(std::string);

	struct drive_assist {

		float Kp = -0.0195f;
		float Ki;
		float Kd = 0.030;
		float i_zone = 0;
		float i_state = 0;
		float prev_err = 0;
		float setpoint = -2;
	} pid;


	int current_pipeline;

	const float ratio =  (515 / (2 * 2 * M_PI));
	const float ticks_per_degree = 979 / 90.11;
	// 2 * ratio * 12 (feet) = ticks

	virtual ~Tracking();
};

#endif /* SRC_VISION_H_ */
