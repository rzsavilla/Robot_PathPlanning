#pragma once
#include "Grid.h"
#include "MapReader.h"	//Creates grid file
#include "AStar.h"		//Pathfinder

class FollowPath : public ArAction // Class action inherits from ArAction
{
	enum State
	{
		Test,Loading,Idle,Rotating, Forward
	};

private: //Odometry
	float m_fX, m_fY, m_fTh;	//Robot position acccording to odometry
	float m_fRX, m_fRY, m_fRTh;	//Robot position according to aria

	double m_tr;	//Wheel radius
	double m_l;	//Wheelbase of the robot
	double m_rw;	//Wheel radius of the robot
	double m_t1, m_t2; // Wheel encoder counts

	void simulateEncoders();

	DWORD m_newTs, m_oldTs;
	float m_fTravelled;			//Distance travelled

	void calcOdometry();
private:
	Grid* m_ptrGrid;
	std::vector<int>* m_ptrviPath;	//Stores grid node indexes
private:
	State m_state;

	Point m_pStartPos;
	Point m_fDesiredPos;

	float m_fDesiredPosX;
	float m_fDesiredPosY;

	float m_fDesiredHeading;

	bool m_bNodeReached;
	bool m_bHeadingSet;
	bool m_bNodeSet;		//Node to move towards has been set

	void init();
public:
	FollowPath(); //!< Constructor
	virtual ~FollowPath() {}  //<! Destructor
	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
	ArActionDesired desiredState; // Holds state of the robot that we wish to action

	void setPath(std::vector<int>* path, Grid* grid);
	void setRobot(float x, float y, float th);

	void moveTo(float x, float y);
};
