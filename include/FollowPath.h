#pragma once
#include "Grid.h"
#include "MapReader.h"	//Creates grid file
#include "AStar.h"		//Pathfinder

class FollowPath : public ArAction // Class action inherits from ArAction
{
	enum State
	{
		Idle,NextNode,Rotating,Forward,GeneratePath
	};

private: //Odometry
	float m_fX, m_fY, m_fTh;	//Robot position acccording to odometry
	float m_fRX, m_fRY, m_fRTh;	//Robot position according to aria

	double m_tr;	//Wheel radius
	double m_l;	//Wheelbase of the robot
	double m_rw;	//Wheel radius of the robot
	double m_t1, m_t2; // Wheel encoder counts

	DWORD m_newTs, m_oldTs;

	void simulateEncoders();
	void calcOdometry();
private:
	Grid* m_ptrGrid;				//!< The grid map for used for planning paths
	std::vector<int>* m_ptrviPath;	//!< Stores grid node indexes
	std::vector<int> m_viVisited;	//!< Stores list of visited nodes by index
private:
	State m_state;				//!< FSM current state

	Point m_pDesiredPos;		//!< Position of desired node
	float m_fDesiredHeading;	//!< Heading towards desired node
	bool m_bInitRotation;		//!< True if initial rotation has been done

	Point m_pGoalPos;			//!< Position of current goal/end of path

	AStar m_pathFinder;		//!< Used to plan paths to random nodes on the grid

	float m_fSpeed;			//!< Robot max velocity
	float m_fRotError;		//!< Used to check if desired heading is achieved
	float m_fMinRandGoal;	//!< Minumum distance from next random goal position
	float m_fDist;			//!< Distance from next node
public:
	FollowPath();				//!< Constructor
	virtual ~FollowPath() {}	//<! Destructor

	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action

	ArActionDesired desiredState; // Holds state of the robot that we wish to action

	//! Set the grid for robot to use in path finding and a path for it to follow
	void setPath(std::vector<int>* path, Grid* grid);
};
