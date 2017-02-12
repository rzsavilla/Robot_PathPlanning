#include <iostream>
#include <stdlib.h>

#include <Aria.h>

#include "avoid.h"

#include <cstdlib>			//rand and srand
#include <time.h>


int normalizeAngle(int angle)
{
	//http://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
	int newAngle = angle;
	while (newAngle <= -180) newAngle += 360;
	while (newAngle > 180) newAngle -= 360;
	return newAngle;
}

// Implementation

int Avoid::getRand(int min, int max)
{
	return min + rand() % (int)(max - min + 1);
}

float addHeading(float current,float angle) {
	float f = current;
	if (current + angle > 180) {
		f = current + angle - 180;
		f = -f;
	}
	else if (current + angle <= -179) {
		f = current + angle + 180;
		f = abs(f);
	}
	else {
		f = current + angle;
	}
	return f;
}

bool Avoid::fastestTurn()
{
	/*http://stackoverflow.com/questions/25506470/how-to-get-to-an-angle-choosing-the-shortest-rotation-direction */
	if (fDesiredAngle < myRobot->getTh()) {
		if (abs(fDesiredAngle - myRobot->getTh()) < 180)
			bLeft = false;
		else bLeft = true;
	}

	else {
		if (abs(fDesiredAngle - myRobot->getTh()) < 180)
			bLeft = true;
		else bLeft = false;
	}

	return bLeft;
}

// Constructor
Avoid::Avoid() : ArAction("Avoid")
{
	state = idle;
	
	fSpeed = 200.0f;
	fTurnSpeed = 10.0f;
	fDesiredAngle = 0.0f;

	fDist = 500.0f;	
	fOffset = 2.0f;	

	bSeeded = false;
	bLeft = false;
}

// Body of action
ArActionDesired * Avoid::fire(ArActionDesired d)
{
	if (!bSeeded) {
		srand(time(NULL));
		bSeeded = true;
	}

	desiredState.reset(); // reset the desired state (must be done)
	//Check obstacles
	float leftSonar = myRobot->getClosestSonarRange(10, 90);
	float rightSonar = myRobot->getClosestSonarRange(-90, -10);
	float frontSonar = myRobot->getClosestSonarRange(-30, 30);

	float fClosest = myRobot->getClosestSonarRange(1.0f, 359.0f);

	float fTurnAngle = 20.0f;

	switch (state)
	{
	case Avoid::idle:
		//Obstacle front
		if (frontSonar <= fDist) {
			//Set robots desired angle to to face behind it
			state = turnAround;
			if (myRobot->getTh() >= 0) fDesiredAngle = myRobot->getTh() - 180;
			else fDesiredAngle = myRobot->getTh() + 180;
			fDesiredAngle = (int)fDesiredAngle % 360;
			if (fDesiredAngle >= 180) {
				fDesiredAngle = 0;
			}
			else if (fDesiredAngle < -180) {
				fDesiredAngle = 0;
			}
			desiredState.setVel(0.0f);
		}
		//Obstacle on the left
		else if (leftSonar < fDist && leftSonar < rightSonar) {
			//Turn right
			fDesiredAngle = addHeading(myRobot->getTh(),-fTurnAngle);	//Set turn angle
			fastestTurn();													//Set turn direction
			state = turnAround;				
		}
		//Obstacle on the right
		else if (rightSonar < fDist && rightSonar < leftSonar) {
			//Turn left
			fDesiredAngle = addHeading(myRobot->getTh(), fTurnAngle);	//Set turn angle
			fastestTurn();													//Set turn direction
			state = turnAround;
		}
		else {
			//Do nothing
		}
		break;
	case Avoid::turnAround:
		//Check if turn is complete
		if (myRobot->getTh() > fDesiredAngle - fOffset && myRobot->getTh() < fDesiredAngle + fOffset) {
			state = idle;
		}
		//Apply turn
		else {
			desiredState.setVel(0.0f);		//Stop moving whilst turning
			if (bLeft) desiredState.setDeltaHeading(fTurnSpeed);
			else desiredState.setDeltaHeading(-fTurnSpeed);
		}
		break;
	default:
		break;
	}
	//switch (state)
	//{
	//case Avoid::idle:
	//	std::cout << "Avoidance State: " << "IDLE" << "\n";
	//	break;
	//case Avoid::turnAround:
	//	std::cout << "Avoidance State: " << "TURNING Avoid obstacle" <<  "\n";
	//	break;
	//default:
	//	break;
	//}

	std::cout << "Velocity: " << myRobot->getVel() << " Heading:" << myRobot->getTh() << " TargetHeading: " << fDesiredAngle << "\n";
	std::cout << "Sonar: " << "Closest: " << fClosest << " Front:" << frontSonar << " Left:" << leftSonar << " Right:" << rightSonar << "\n";

	return &desiredState; // give the desired state to the robot for actioning
}