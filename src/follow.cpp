#include <iostream>
#include <stdlib.h>
#include <Aria.h>
#include <time.h>
#include <ctime>

#include "follow.h"
// Implementation

// Constructor
follow::follow() : ArAction("Follow Edge")
{
	speed = 200.0f; // Set the robots speed to 50 mm/s. 200 is top speed
	deltaHeading = 0; // Straight line

	setPoint = 700.0f; // 0.7 m

	// Proportional control
	pGain = 0.10; // CHANGE THIS
	iGain = 0.0001f;
	dGain = 1.6f;

	dState = 0.0f;
	pos = 0.0f;
}

// Body of action
ArActionDesired * follow::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)

	// Get sonar readings
	leftSonar = myRobot->getClosestSonarRange(0.0f, 100);
	rightSonar = myRobot->getClosestSonarRange(-100, 0.0f);
	float fClosest = myRobot->getClosestSonarRange(1.0f, 359.0f);

	if (leftSonar <= 1000.0f || rightSonar <= 1000.0f) {

		// Find error
		if (leftSonar < rightSonar) error = leftSonar - setPoint;
		else error = setPoint - rightSonar;

		errors.push_back(error);		//Store errors

		//Compute RMSE
		rmse = 0;	//Root Mean Squared Error
		double sum = 0;
		for (int i = 0; i < errors.size(); i++) {
			sum += pow((errors.at(i) - setPoint), 2);
		}
		sum = sum / errors.size();
		rmse = sqrt(sum) - setPoint;

		//Calculate integral state
		iState += error;
		double iMin, iMax;
		iMin = -1000.0f;
		iMax = 1000.0f;
		if (iState > iMax) iState = iMax;
		else if (iState < iMin)iState = iMin;

		if (leftSonar < rightSonar) pos = leftSonar;
		else pos = rightSonar;

		pTerm = pGain * error;				//Gain * current error
		iTerm = iGain * iState;
		dTerm = dGain * (error - dState);	//Gain * slope of error

		dState = error;
		if (leftSonar < rightSonar) dState = leftSonar - setPoint;
		else dState = rightSonar;

		//PID output
		output = pTerm + iTerm + dTerm;

		// Implement control action
		deltaHeading = output;

		std::cout << "State: FOLLOW\n";
		std::cout << "Sonar: \tClosest:" << fClosest << "\tLeft:" << leftSonar << "\tRight:" << rightSonar << "\n";
		std::cout << "Error: " << error << "\tRMSE:" << (int)rmse << "\n";
		desiredState.setDeltaHeading(deltaHeading); // Set the heading change of the robot
	}
	else {
		std::cout << "State: FORWARD\n";
		std::cout << "Sonar: \tClosest:" << fClosest << "\tLeft:" << leftSonar << "\tRight:" << rightSonar << "\n";
	}

	std::cout << "Velocity: " << myRobot->getVel() << "\tHeading:" << myRobot->getTh() << "\n";
	std::cout << "Position: X:" << myRobot->getX() << "\tY:" << myRobot->getY() << "\n";

	// set the speed of the robot in the desired state
	desiredState.setVel(speed); 
	

	return &desiredState; // give the desired state to the robot for actioning
}


