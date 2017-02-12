#pragma once

// Signatures

class Avoid : public ArAction // Class action inherits from ArAction
{
	enum State
	{
		turnAround,
		idle,
	};

private:
	State state;		//FSM State
	
	float fSpeed;			//Robot move speed
	float fTurnSpeed;		//Robot turn speed
	float fDesiredAngle;	//Robot target heading

	
	float fDist;		//Distance from obstacle
	float fOffset;		//Heading angle offset

	bool bSeeded;		//Random seed generated
	bool bLeft;			//Direction for random turn

	int getRand(int min, int max);	//Return random number
	bool fastestTurn();	//Calculate fastest turn direction towards desired angle
public:
	Avoid(); // Constructor
	virtual ~Avoid() {}  // Destructor
	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
	ArActionDesired desiredState; // Holds state of the robot that we wish to action

	
};
