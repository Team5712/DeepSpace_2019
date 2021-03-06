#include <vision/Tracking.h>

Tracking::Tracking() {

	map<char, float> vision_tape = { { 'a', 5.98428 }, { 'b', 0.566859 }, { 'c', 0.632809 } };

	map<char, float> cargo = { { 'a', 0.3 }, { 'b', 0.0 }, { 'c', 0.0 } };

	Kw.push_back(vision_tape);
	Kw.push_back(cargo);
}

vector<float> Tracking::getTurnAdjustmentPercents(int camera, int pipeline) {

	this->setPipeline(pipeline);
	std::shared_ptr<NetworkTable> table;

	if(camera == 0) {
		table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-one");

	} else if(camera == 1) {
		table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-two");
	} else {
		cout << "Camera not specified " << camera << endl;
	}
	float targetA = table->GetNumber("ta", 0);
	float targetX = table->GetNumber("tx", 0);
	float headingError = -targetX;
	float steeringAdjust = 0.0f;
	float minCommand = 0.04;
	float Kp = -0.0028f; //.0075
	float leftCommand;
	float rightCommand;

	
    // error is the distance from where we want to go from where we are now
    float error = pid.setpoint - targetX;
    // cout << "pot " << pot->Get() << endl;

    // calculate proportion value
    float p = pid.Kp * error;

    // i_zone for perfecting distance to target
    if(fabsf(error) <= pid.i_zone || pid.i_zone == 0.0f) {
        pid.i_state = pid.i_state + (error * pid.Ki);
    } else {
        pid.i_state = 0;
    }

    float d = (error - pid.prev_err);
    pid.prev_err = error;
    d *= pid.Kd;

    float output = p + pid.i_state + d;


	if (targetX < -3.5) {
		steeringAdjust = output - minCommand;
		// cout << "target to the left " << endl;
	} else if (targetX > -1.5) {
		// cout << "target to the right " << endl;
		steeringAdjust = output + minCommand;
	}
	//CHANGE SIGNS NEED TO TEST
	leftCommand -= steeringAdjust;
	rightCommand += steeringAdjust;

	vector<float> adjustments;
	adjustments.push_back(leftCommand);
	adjustments.push_back(rightCommand);
	return adjustments;
}

vector<float> Tracking::getTurnAdjustmentTicks(int camera, int pipeline) {

	// this->setPipeline(pipeline);
	std::shared_ptr<NetworkTable> table;

	if(camera == 0) {
		table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-one");

	} else if(camera == 1) {
		table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-two");
	} else {
		cout << "Camera not specified " << camera << endl;
	}

	float targetX = table->GetNumber("tx", 0);

	float l_value;
	float r_value;

	if(targetX > .75) {
		// distance to setpoint + (tick/degree) * degrees to target
		// divide by  / cos(targetX)
		l_value = (ticks_per_degree * targetX);
		r_value = -(ticks_per_degree * targetX);

	} else if (targetX < -.75) {
		l_value = (ticks_per_degree * targetX);
		r_value = -(ticks_per_degree * targetX);
	}
	 
	vector<float> adjustments;
	adjustments.push_back(l_value);
	adjustments.push_back(r_value);
	return adjustments;
} 

// net distance in inches
float Tracking::getDriveFromSetPointTicks(float net_distance) {
	std::shared_ptr<NetworkTable> table =
		nt::NetworkTableInstance::GetDefault().GetTable("limelight-two");

	//float targetA = table->GetNumber("ta", 0);

	// y = a*b^x+c
	float targetA = table->GetNumber("ta", 0);

	
	//const float distance = 304 / targetW;
	float distance = log((targetA-.0344202)/1.68221)/log(.759274)*asin(.86);
	//float xOffset = sqrt(pow(log((targetA-.0344202)/1.68221)/log(.759274)*asin(.86),2)-pow(log((targetA-.0344202)/1.68221)/log(.759274),2));
	//return (distance * 12 * ratio) - (net_distance * ratio);
	//return distance;
	//return xOffset;
	cout << "dist " << distance << endl;
	return distance;
}

void Tracking::setPipeline(int pipeline) {

	cout << "pipeline " << pipeline << endl;

	if(pipeline != current_pipeline) {
		// TODO: add this or test it
		// frc::Wait(25);
		nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", pipeline);
		nt::NetworkTableInstance::GetDefault().GetTable("limelight-one")->PutNumber("pipeline", pipeline);
	}
	
	current_pipeline = pipeline;
}

void Tracking::logLimelightValue(string s) {
	std::shared_ptr<NetworkTable> table_one =
	nt::NetworkTableInstance::GetDefault().GetTable("limelight-one");

	std::shared_ptr<NetworkTable> table_two =
	nt::NetworkTableInstance::GetDefault().GetTable("limelight-two");

	cout << "one-" << s << " " << table_one->GetNumber(s, 0) << endl;
	cout << "two-" << s << " " << table_two->GetNumber(s, 0) << endl;
}

Tracking::~Tracking() {
	// TODO Auto-generated destructor stub
}

