#pragma once
#include <vector>

// Signatures

class follow : public ArAction // Class action inherits from ArAction
{
public:
	follow(); // Constructor
	virtual ~follow() {}  // Destructor
	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
	ArActionDesired desiredState; // Holds state of the robot that we wish to action
protected:
	int speed; // Speed of the robot in mm/s
	double deltaHeading; // Change in heading
						 // Reading
	double leftSonar;
	double rightSonar;

	// Control variables
	double setPoint; // Set point of the controller
	double error; // Current error
	std::vector<double> errors;
	double output; // Final output signal
	double rmse;	//Root mean squared error
	double pGain; // Gain
	double iGain;
	double dGain;
	double pTerm = 0;
	double iTerm = 0;
	double dTerm = 0;

	double iState = 0;
	double dState = 0;		//Last position

	float pos;

};
