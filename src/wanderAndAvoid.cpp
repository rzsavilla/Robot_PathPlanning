#include <iostream>
#include <stdlib.h>

#include <Aria.h>

#include "wanderAndAvoid.h"

#include <cstdlib>			//rand and srand
#include <time.h>

// Implementation

int FSM::getRand(int min, int max)
{
	return min + rand() % (int)(max - min + 1);
}

// Constructor
FSM::FSM() : ArAction("FSM")
{
	state = forward;

	bool bTurning = false;
	fSpeed = 200.0f;
	fTurnSpeed = 10.0f;
	fDesiredAngle = 45.0f;
	iTick = 0;
	iRandTick = 0;

	srand(time(NULL));	//Generate random seed
}

// Body of action
ArActionDesired * FSM::fire(ArActionDesired d)
{
 desiredState.reset(); // reset the desired state (must be done)

 //Check obstacles
 float leftSonar = myRobot->getClosestSonarRange(10, 90);
 float rightSonar = myRobot->getClosestSonarRange(-90, -10);
 float frontSonar = myRobot->getClosestSonarRange(-30, 30);

 float fDist = 500.0f;	//Distance from obstacle

 int iTurnTimeout = 0;	//Prevent stuck at turning

 switch (state)
 {
 case FSM::forward:
	 std::cout << "front:" << frontSonar << " left:" << leftSonar << " right:" << rightSonar << "\n";
	 if (frontSonar < fDist) {
		 state = turnAround;
		 if (myRobot->getTh() >= 0) fDesiredAngle = myRobot->getTh() - 180;
		 else fDesiredAngle = myRobot->getTh() + 180;
		 //fDesiredAngle = myRobot->getTh() - 180;
		 fDesiredAngle = (int)fDesiredAngle % 360;
		 if (fDesiredAngle >= 180) {
			 fDesiredAngle = 0;
		 }
		 else if (fDesiredAngle < -180) {
			 fDesiredAngle = 0;
		 }
		 desiredState.setVel(0.0f);
	 }
	 else if (leftSonar < fDist && leftSonar < rightSonar) {
		 //obstacle on the left
		 state = turnRight;
		 desiredState.setVel(0.0f);
	 }
	 else if (rightSonar < fDist && rightSonar < leftSonar) {
		 //Obstacle on the right
		 state = turnLeft;
		 desiredState.setVel(0.0f);
	 }
	 else {
		 //Move Forward
		 state = forward;
		 desiredState.setVel(fSpeed);
	 }
	 break;
 case FSM::turnLeft:
	 if (iTick < 20) {
		 desiredState.setDeltaHeading(fTurnSpeed);
		 iTick++;
	 }
	 else {
		 iTick = 0;
		 state = forward;
	 }
	 break;
 case FSM::turnRight:
	 if (iTick < 20) {
		 desiredState.setDeltaHeading(-fTurnSpeed);
		 iTick++;
	 }
	 else {
		 iTick = 0;
		 state = forward;
	 }
	 break;
 case FSM::randomTurn:
	 fDesiredAngle = getRand(-180, 180);
	 state = turnAround;
	 break;
 case FSM::turnAround:
	 std::cout << "th: "<< myRobot->getTh() << "\t Target: " << fDesiredAngle << "\n";
	 if (myRobot->getTh() > fDesiredAngle - 1 && myRobot->getTh() < fDesiredAngle + 1) {
		 state = forward;
		 iTurnTimeout = 0;
	 }
	 else {
		 desiredState.setDeltaHeading(fTurnSpeed);
		 iTurnTimeout++;
	 }

	 break;
 default:
	 break;
 }

 if (iRandTick > 500) {
	 //Random turn
	 state = randomTurn;
	 iRandTick = 0;
 }
 iRandTick++;

 switch (state)
 {
 case FSM::forward:
	 std::cout << "State: " << "FORWARD" << "\n";
	 break;
 case FSM::turnLeft:
	 std::cout << "State: " << "TURN LEFT" << "\n";
	 break;
 case FSM::turnRight:
	 std::cout << "State: " << "TURN RIGHT" << "\n";
	 break;
 case FSM::randomTurn:
	 std::cout << "State: " << "RANDOM TURN" << "\n";
	 break;
 case FSM::turnAround:
	 std::cout << "State: " << "TURN AROUND" << "\n";
	 break;
 default:
	 break;
 }

 return &desiredState; // give the desired state to the robot for actioning
}