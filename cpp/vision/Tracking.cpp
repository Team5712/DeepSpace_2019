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
	float targetX = table->GetNumber("tx", 0); //This caluates the x angle if camera was centered
	//To calulate aDistance, bDistance, and cDistance. Go to Desmos, make a table, plot the distance on the left and ta value on right side.
	/*float aDistance=4.92023;
	float bDistance=.588173;
	float cDistance=.133908;
	float distanceAdjust=1; //Lower value to get tower distance if constantly high. Raise distance to get larger distance
	//To calulate aAngle, bAngle, cAngle. Go to Desmos, make a table, plot the distance on the left and tx value on right side.
	float aAngle=-25.2663;
	float bAngle=.648492;
	float cAngle=-2.22648;
	float angleAdjust=2; //Lower value to turn farther one way and raise to go farther way.
	float distance = log((targetA-cDistance)/aDistance)/log(bDistance)*distanceAdjust; //Calculates distance to targer
	float targetX=(aAngle*pow(bAngle,distance)+cAngle)*angleAdjust; //Calculate what angle it should be at */
	float headingError = -targetX;
	float steeringAdjust = 0.0f;
	float minCommand = 0.04;
	float Kp = -0.0075f;
	float leftCommand;
	float rightCommand;
	//cout << "targerA" << targetA << endl;
	cout << "targetX" << targetX << endl;
	//cout << "targetXCenter" << targetXCenter << endl;
	//cout << "Angle to Turn" << targetX << endl;
	/*cout << "Distance" << distance << endl;
	cout << "Calculated Angle" << targetX <<endl;
	cout << "Limelight Angle" << targetXCenter <<endl;
	//cout << "Offset" << xOffset << endl; */
	if (targetX < -1) {
		steeringAdjust = Kp * headingError - minCommand;
	} else if (targetX > 1) {
		steeringAdjust = Kp * headingError + minCommand;
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

	this->setPipeline(pipeline);
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
	

	
	//const float distance = 304 / targetW;
	//float distance = log((targetA-.0344202)/1.68221)/log(.759274)*asin(.86);
	//float xOffset = sqrt(pow(log((targetA-.0344202)/1.68221)/log(.759274)*asin(.86),2)-pow(log((targetA-.0344202)/1.68221)/log(.759274),2));
	//return (distance * 12 * ratio) - (net_distance * ratio);
	//return distance;
	//return xOffset;
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

