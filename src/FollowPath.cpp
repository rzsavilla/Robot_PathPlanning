#include <iostream>
#include <stdlib.h>

#include <Aria.h>

#include "FollowPath.h"

#include <cstdlib>			//rand and srand
#include <time.h>

static const double PI = 3.14159265359;

void FollowPath::simulateEncoders()
{
	m_newTs = GetTickCount();

	long long unsigned int oldt = (long long unsigned int)m_oldTs;   // Last time click as 64bit 
	long long unsigned int newt = (long long unsigned int)m_newTs;  // New time click as 64bit 
	int diff = newt - oldt; // Time since last calculation

	double d1 = ((double)diff * myRobot->getLeftVel()) / 1000.0f; // D = speed * time / 1000000.0 to get into seconds
	double d2 = ((double)diff * myRobot->getRightVel()) / 1000.0f;

	m_t1 = floor((d1 * m_tr) / (2.0f * 3.14f * m_rw)); // Rearranged odmetry equation, round as ticks would always be ints
	m_t2 = floor((d2 * m_tr) / (2.0f * 3.14f * m_rw));

	m_oldTs = m_newTs; // Set old time
}

void FollowPath::calcOdometry()
{
	//Aria readings
	//m_fRX = myRobot->getX();
	//m_fRY = myRobot->getY();
	m_fRTh = myRobot->getTh(); // (in degrees)

	simulateEncoders();

	double dist = 0.0f;
	double dx = 0.0f;
	double dy = 0.0f;
	double dTh = 0.0f;

	float fRadians = (m_fRTh) * PI / 180.0f;	//Convert to radians

	dx = cos(fRadians) * (m_t1 + m_t2) * ((PI * m_rw) / m_tr);
	dy = sin(fRadians) * (m_t1 + m_t2) * ((PI * m_rw) / m_tr);

	if (myRobot->getVel() > 0) {
		m_fX += dx;
		m_fY += dy;
	}
}

FollowPath::FollowPath() : ArAction("FollowPath")
{
	m_fSpeed = 200.0f;	//Set to max speed 0.2m/s
	m_pathFinder.addTraversable(2, 0, 3);
	m_pathFinder.setMovementCost(5.0f, 10.0f);
	m_bInitRotation = false;

	m_fRotError = 2.0f;
	m_fMinRandGoal = 500.0f;

	m_state = State::Idle;
}

ArActionDesired * FollowPath::fire(ArActionDesired d)
{
	desiredState.reset();

	m_fX = myRobot->getX();
	m_fY = myRobot->getY();
	m_fTh = myRobot->getTh();

	calcOdometry();

	//Distance to the closest object
	float fNearest = myRobot->getClosestSonarRange(1.0f, 359.0f);

	std::cout << "State: ";
	switch (m_state)
	{
	case State::Idle: {
		std::cout << "Idle\n";
		if (m_ptrGrid != NULL) {
			if (!m_ptrviPath->empty()) {
				m_state = State::NextNode;		//Follow Path
			}
			else {
				//No path to follow
				m_bInitRotation = false;		//Initial rotation required for next path planned
				m_state = State::GeneratePath;	//Choose random node and plan path
			}
		}
		break;
	}
	case State::NextNode: {
		/*
			Sets which node to move towards when following a path
		*/
		std::cout << "Next Node\n";
		if (!m_ptrviPath->empty()) {
			//Set Node to move towards
			m_pGoalPos = m_ptrGrid->vNodes.at(m_ptrviPath->back())->m_pMapCoord;		//Goal position last node in path
			m_pDesiredPos = m_ptrGrid->vNodes.at(m_ptrviPath->front())->m_pMapCoord;	//Position of next node in path
			//Calculate heading required to move towards node
			m_fDesiredHeading = myRobot->findAngleTo(ArPose(m_pDesiredPos.x, m_pDesiredPos.y));

			if (m_bInitRotation) {
				m_state = State::Forward;	//Move towards node
			}
			else {
				m_state = State::Rotating;	//Perform initial rotation
			}
		}
		else {
			m_state = State::Idle;
		}
		break;
	}
	case State::Rotating: {
		/*
		Ensures robot is facing the path before moving
		preventing crash from movement and turning at the same time
		*/
		std::cout << "Rotating\n";
		//Stop the robot and apply rotation
		desiredState.setVel(0.0f);			//Stop
		m_fDesiredHeading = myRobot->findAngleTo(ArPose(m_pDesiredPos.x, m_pDesiredPos.y));
		desiredState.setHeading(m_fDesiredHeading);		
		
		//Check if desired heading has been reached
		if (m_fTh > m_fDesiredHeading - m_fRotError && m_fTh < m_fDesiredHeading + m_fRotError) {
			m_bInitRotation = true;		//Initial rotation has been achieved
			m_state = State::Forward;	//Robot can now move forward
		}
		break;
	}
	case State::Forward: {
		std::cout << "Forward\n";
		m_fDesiredHeading = myRobot->findAngleTo(ArPose(m_pDesiredPos.x, m_pDesiredPos.y));	//Update Heading
		//Turn towards heading
		desiredState.setHeading(m_fDesiredHeading);	
		//Move Forward
		desiredState.setVel(m_fSpeed);

		//Check if desired position has been reached
		float fDistance = sqrt(pow(m_pDesiredPos.x - m_fX, 2) + pow(m_pDesiredPos.y - m_fY, 2));
		m_fDist = fDistance;
		if (abs(fDistance) < m_fSpeed) {
			m_state = State::NextNode;					//Node Reached
			m_ptrviPath->erase(m_ptrviPath->begin());	//Remove node
		}
		break;
	}
	case State::GeneratePath: {
		std::cout << "GeneratingPath\n";

		//Generate random index to determine node to move towards on the grid.
		int iIndex = rand() % (m_ptrGrid->vNodes.size() - 1);
		Point pNewGoalPos = m_ptrGrid->vNodes.at(iIndex)->m_pMapCoord;

		bool bVisited = false;
		//Check if node has been visited
		for (auto it = m_viVisited.begin(); it != m_viVisited.end(); ++it) {
			if ((*it) == iIndex) {
				bVisited = true;
				break;
			}
		}
		int iCurrIndex = getIndex(floor(m_fX / m_ptrGrid->iCellSize), floor(m_fY / m_ptrGrid->iCellSize), m_ptrGrid->uiWidth);
		std::cout << "iC:" << iCurrIndex << "Index: " << iIndex << "\n";
	
		if (!bVisited) {
			//Calculate distance between current position and random goal position
			float fDist = sqrt(pow(pNewGoalPos.x - m_fX, 2) + pow(pNewGoalPos.y - m_fY, 2));
			if (fDist > m_fMinRandGoal) {
				//Plan path
				if (m_pathFinder.generatePath(Point(m_fX, m_fY), pNewGoalPos, m_ptrGrid, m_ptrviPath)) {
					//Save path
					m_state = State::Idle;
					m_viVisited.push_back(iIndex);
				}
			}
		}
		break;
	}
	default:
		break;
	}

	std::cout << "Current Pos X:" << m_fX << " Y:" << m_fY << " th:" << m_fTh << " Vel:" << myRobot->getVel() << "\n";
	std::cout << "Next    Pos X:" << m_pDesiredPos.x << " Y:" << m_pDesiredPos.y << " th:" << m_fDesiredHeading << " Dist:" << m_fDist << "\n";
	std::cout << "Goal    Pos X:" << m_pGoalPos.x << " Y:" << m_pGoalPos.y << " RemainingNodes:" << m_ptrviPath->size() << " Closest Object: " << fNearest << "\n";
	return &desiredState;
}

void FollowPath::setPath(std::vector<int>* path, Grid * grid)
{
	std::srand((unsigned)time(NULL));
	m_ptrviPath = path;
	m_ptrGrid = grid;

	m_l = 425.0; // Width of a P3 DX in mm
	m_rw = 95.0; // Wheel radius in mm
	m_tr = 360.0; // Number of odemetry clicks for a full circle

	m_fX = m_ptrGrid->pMapStart.x;
	m_fY = m_ptrGrid->pMapStart.y;

	myRobot->setEncoderTransform(ArPose(m_ptrGrid->pMapStart.x, m_ptrGrid->pMapStart.y, m_ptrGrid->fStartTh));
	m_bInitRotation = false;
}