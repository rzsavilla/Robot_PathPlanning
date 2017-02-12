#pragma once
// Signatures

class Wander : public ArAction // Class action inherits from ArAction
{
	enum State
	{
		forward,wander, turnLeft, turnRight, randomTurn, turnAround
	};

private:
	int getRand(int min, int max);

	//Odometry
	void simulateEncoders();
	DWORD newTs, oldTs;
	double robotX, robotY, robotTh; // Robots position according to Aria
	double myX, myY, myTh; // Robots position according to your odometry calculations
	double t1, t2; // Wheel encoder counts
	double tr; // Radius of the wheel
	double l; // Wheelbase of the robot
	double rw; // Wheel radius of the robot

	void updateOdometry();

	float fDistTravelled;	//Distance travelled
	float fTravelDist;		//Distance to travel

	bool bLeft;			//Direction for random turn
	bool bSeeded;		//Random seed has been generated
	bool bAngleSet;		//Random desired angle set

	float getRandomTurn(float min, float max);

public:
	Wander(); // Constructor
	virtual ~Wander() {}  // Destructor
	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
	ArActionDesired desiredState; // Holds state of the robot that we wish to action

	State state;

	bool bTurning;
	float fSpeed;
	float fTurnSpeed;
	float fDesiredAngle;

	int iTurnTimeout;

	int iTick;
	int iRandTick;
};
