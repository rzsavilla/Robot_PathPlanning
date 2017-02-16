#include <iostream>
#include <stdlib.h>

#include <Aria.h>

#include "FollowPath.h"

#include <cstdlib>			//rand and srand
#include <time.h>

void FollowPath::init()
{
	//////////	Generate Grid Map	///////////
	MapReader mapReader;		//Create grid using map file
	std::string sMapResLoc = "resources/maps/";
	std::string sGridResLoc = "resources/grids/";
	std::string sMapName = "Mine";

	std::cout << "Generating Grid\n";
	std::string sMapFile = (sMapResLoc.append(sMapName)).append(".map");
	mapReader.createGrid(sMapFile, &m_grid, 100);
	

	m_pStartPos = mapReader.m_pStartPos;

	AStar pathFinder;
	pathFinder.addTraversable(1, 0);
	pathFinder.getPath(mapReader.m_pStartPos, mapReader.m_pGoalPos, &m_grid, &m_viPath);

	mapReader.saveGrid(&m_grid, "resources/grids/" + sMapName + ".txt");
}

FollowPath::FollowPath() : ArAction("FollowPath")
{
	m_state = State::Loading;
}

ArActionDesired * FollowPath::fire(ArActionDesired d)
{
	desiredState.reset();
	m_fX = myRobot->getX() + m_grid.pStartPos.x;
	m_fY = myRobot->getY() + m_grid.pStartPos.y;
	m_fTh =  myRobot->getTh() ;
	//myRobot->set
	float fOffset = 1.0f;
	float m_fRotateHeading;
	float fPositionOffset = 100.0f;
	Point pDiff;
	float fDistance = 0;
	switch (m_state)
	{
	case Idle: {
		if (!m_viPath.empty()) {
			m_fDesiredPos = m_grid.vNodes.at(m_viPath.front())->m_pMapCoord;
			//Calculate heading towards desired position
			float fDiff; //Angle between two points
			fDiff = atan2(m_fDesiredPos.y - m_fY, m_fDesiredPos.x - m_fX) * 180 / 3.14159265359;	//Angle in degrees
			fDiff -= m_grid.fStartTh;
			m_fDesiredHeading = (int)(fDiff) % 360;
			if (m_fDesiredHeading > 180) m_fDesiredHeading -= 180;

			m_viPath.erase(m_viPath.begin());
			m_state = Rotating;
			m_bNodeSet = true;
		}
		else {
			std::cout << "Goal Reached\n";
		}
		break;
	}
	case FollowPath::Rotating: {
		desiredState.setVel(0);
		float fAngleDiff = m_fDesiredHeading - m_fTh;
		if (fAngleDiff > -fOffset && fAngleDiff < fOffset) {
			m_state = State::Forward;
		}
		else {
			if (fAngleDiff < 0) desiredState.setDeltaHeading(-10);
			else desiredState.setDeltaHeading(10);
		}
		break;
	}
	case FollowPath::Forward: {
		
		//Check if desired position has been reached
		pDiff = Point(m_fDesiredPos.x - m_fX, m_fDesiredPos.x - m_fY);
		fDistance = sqrt(pow(pDiff.x, 2) + pow(pDiff.y, 2));
		fDistance = sqrt(pow(m_fDesiredPos.x - m_fX, 2) + pow(m_fDesiredPos.y - m_fY,2));
		
		std::cout << fDistance << "\n";
		if (fDistance < 250) {
			m_state = State::Idle;
		}

		//Move Forward
		desiredState.setVel(200);
		break;
	}
	case Loading: {
		init();
		m_state = State::Idle;
		//myRobot->setHeading(m_grid.fStartTh);
	}
	default:
		break;
	}


	std::cout << "X:" << m_fX << " Y:" << m_fY << " th:" << m_fTh << "\n";
	std::cout << m_fDesiredPos.x << " " << m_fDesiredPos.y << " " << fDistance << " " << m_fDesiredHeading << "\n";


	return &desiredState;
}

void FollowPath::setRobot(float x, float y, float th)
{
	m_fTh = myRobot->getTh();
	m_fX = myRobot->getX() + x;
	m_fY = myRobot->getY() + y;

	m_fTh = myRobot->getTh();
	m_fTh = (int)(m_fTh + th) % 360;
	if (m_fTh > 180) m_fTh -= 180;
}

void FollowPath::moveTo(float x, float y)
{
	//Find angle towards point
	float fPosX = 0.0;
	float fPosY = 0.0;
	float fThe = 0.0f;

	float fAngleDiff;

	float dot = fPosX * x + fPosY * y;
	float fMag1 = sqrt(pow(fPosX, 2) + pow(fPosY, 2));
	float fMag2 = sqrt(pow(x, 2) + pow(y, 2));

	fAngleDiff = cos(dot / (fMag1 * fMag2));
}
