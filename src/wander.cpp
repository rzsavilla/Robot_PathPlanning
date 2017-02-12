#include <iostream>
#include <stdlib.h>

#include <Aria.h>

#include "wander.h"

#include <cstdlib>			//rand and srand
#include <time.h>

// Implementation

int Wander::getRand(int min, int max)
{
	return min + rand() % (int)(max - min + 1);
}

float Wander::getRandomTurn(float min, float max)
{
	fDesiredAngle = getRand(min, max);
	//Fastest turn direction towards angle
	float f = 0.0f;

	if (fDesiredAngle > 0) f = fDesiredAngle - myRobot->getTh();
	else f = myRobot->getTh() + fDesiredAngle;

	if (f > 0) bLeft = true;	//Turn left
	else bLeft = false;			//Turn right
	bAngleSet = true;
	return fDesiredAngle;
}

// Constructor
Wander::Wander() : ArAction("Wander")
{
	state = forward;
	bSeeded = false;

	bool bTurning = false;
	fSpeed = 200.0f;
	fTurnSpeed = 10.0f;
	fDesiredAngle = 0.0f;
	iTick = 0;
	iRandTick = 0;

	//Odometry
	myX = 0.0; // init x pos
	myY = 0.0; // init y pos
	myTh = 0.0; // init heading

	l = 425.0; // Width of a P3 DX in mm
	rw = 95.0; // Wheel radius in mm
	tr = 360.0; // Number of odemetry clicks for a full circle

	fDistTravelled = 0.0f;
	fTravelDist = 0.0f;

	bLeft = false;

}

// Body of action
ArActionDesired * Wander::fire(ArActionDesired d)
{
	if (!bSeeded) {
		srand(time(NULL));	//Generate random seed
		bSeeded = true;
	}

	desiredState.reset(); // reset the desired state (must be done)

	//Check obstacles
	float fClosest = myRobot->getClosestSonarRange(1.0f, 359.0f);

	float fOffset = 2.0f;	//Heading offset

	updateOdometry();

	switch (state)
	{
	case Wander::forward:

		//Wander
		if (fClosest > 1500.0f) {
			//Generate random distance to travel
			fTravelDist = getRand(500.0f, 1500.0f);
			
			desiredState.setVel(fSpeed);
			state = wander;
		}
		else {
			//Move Forward
			desiredState.setVel(fSpeed);
		}

		break;
	case Wander::wander:
		//Moves forward until specified distance has been reached
		if (fDistTravelled >= fTravelDist) {
			//Calculate random heading
			fDistTravelled = 0.0f;					//Reset distance travelled
			getRandomTurn(0.0f, 140.0f);		//Sets desired turn
			state = randomTurn;
		}
		else {
			desiredState.setVel(fSpeed);
		}
		break;
	case Wander::randomTurn:
		//Check if turn is complete
		if (myRobot->getTh() > fDesiredAngle - fOffset && myRobot->getTh() < fDesiredAngle + fOffset) {
			state = forward;	//Turn complete
		}
		//Apply Turn
		else {
			desiredState.setVel(0.0f);									//Stop moving
			if (!bLeft) desiredState.setDeltaHeading(-fTurnSpeed);		//Turn Left
			else desiredState.setDeltaHeading(fTurnSpeed);				//Turn Right
		}
		break;
	default:
		break;
	}

	switch (state)
	{
	case Wander::forward:
		std::cout << "State: " << "FORWARD" << "\n";
		break;
	case Wander::wander:
		std::cout << "State: " << "WANDER" << "\n";
		break;
	case Wander::randomTurn:
		std::cout << "State: " << "RANDOM TURN" << "\n";
		break;
	default:
		break;
	}

	std::cout << "Velocity: " << myRobot->getVel() << " Heading:" << myRobot->getTh() << " TargetHeading: " << fDesiredAngle << "\n";
	std::cout << "Position: X:" << myRobot->getX() << "\tY:" << myRobot->getY() << "\n";
	std::cout << "Distance Travelled: " << fDistTravelled << "\tDistance Target: " << fTravelDist << "\n";
	
	return &desiredState; // give the desired state to the robot for actioning
}


void Wander::simulateEncoders()
{
	newTs = GetTickCount();

	long long unsigned int oldt = (long long unsigned int)oldTs;   // Last time click as 64bit 
	long long unsigned int newt = (long long unsigned int)newTs;  // New time click as 64bit 
	int diff = newt - oldt; // Time since last calculation

	double d1 = ((double)diff * myRobot->getLeftVel()) / 1000.0f; // D = speed * time / 1000000.0 to get into seconds
	double d2 = ((double)diff * myRobot->getRightVel()) / 1000.0f;

	t1 = floor((d1 * tr) / (2.0f * 3.14f * rw)); // Rearranged odmetry equation, round as ticks would always be ints
	t2 = floor((d2 * tr) / (2.0f * 3.14f * rw));

	oldTs = newTs; // Set old time
}

void Wander::updateOdometry()
{
	//Odometry
	simulateEncoders();

	robotTh = myRobot->getTh(); // (in degrees)

	float Pi = 3.14159265359;
	double dist = 0;
	double dx = 0;
	double dy = 0;
	double dTh = 0;

	float radians = myRobot->getTh() * Pi / 180;

	dx = cos(radians) * (t1 + t2) * ((Pi * rw) / tr);
	dy = sin(radians) * (t1 + t2) * ((Pi * rw) / tr);

	dTh = radians + (2 * Pi) * (t2 - t1) * (rw / (tr *l));


	myTh = myTh + dTh;
	if (myRobot->getVel() > 0.0f) {
		myX = myX + dx;
		myY = myY + dy;
		fDistTravelled += abs(dx);
		fDistTravelled += abs(dy);
	}
}