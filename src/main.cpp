#include "Aria.h"
#include "wanderAndAvoid.h"
#include "wander.h"
#include "avoid.h"
#include "follow.h"
#include "MapReader.h"
#include "AStar.h"
#include "FollowPath.h"

void moveTo(float x, float y, Grid* grid, std::vector<int>* viPath)
{
	//Find angle towards point
	float fPosX = 0.0;
	float fPosY = 0.0;
	float fTh = 0.0f;
	float fTargetAngle = 0.0f;

	float dot = fPosX * x + fPosY * y;						//Dot product of the two points
	float fMag1 = sqrt(pow(fPosX, 2) + pow(fPosY, 2));	
	float fMag2 = sqrt(pow(x, 2) + pow(y, 2));

	float fAngleDiff; //Angle between two points
	fAngleDiff = atan2(y - fPosY, x - fPosX) * 180 / 3.14159265359;	//Angle in degrees
	fTargetAngle = (int)(fTh + fAngleDiff) % 360;
	if (fTargetAngle > 180) fTargetAngle -= 180;

	//Remove offset
	for (auto it = grid->vNodes.begin(); it != grid->vNodes.end(); ++it) {
		//Grid coordinates to map coordinates
		(*it)->m_pMapCoord.x -= grid->pOffset.x;
		(*it)->m_pMapCoord.y -= grid->pOffset.y;
	}

	//Iterate through paths
	for (int i = 0; i < viPath->size(); i++) {
		float fTargetX = (grid->vNodes.at(viPath->at(i))->m_pGridCoord.x * grid->iCellSize) / 100;
		float fTargetY = (grid->vNodes.at(viPath->at(i))->m_pGridCoord.y * grid->iCellSize) / 100;
		std::cout << "X:" << fTargetX << " Y:" << fTargetY << "\n";
	}
}


int main(int argc, char **argv)
{
	//////////	Generate Grid Map	///////////
	MapReader mapReader(100,4);		//Create grid using map file
	std::string sMapResLoc = "resources/maps/";
	std::string sGridResLoc = "resources/grids/";
	std::string sMapName = "Mine";

	Grid grid_map;
	std::cout << "Generating Grid\n";
	std::string sMapFile = (sMapResLoc.append(sMapName)).append(".map");
	mapReader.createGrid(sMapFile, &grid_map);
	
	std::vector<int> viPath;	//Stores node indexes
	AStar pathFinder;
	pathFinder.addTraversable(1, 0);
	pathFinder.getPath(grid_map.pGridStart, grid_map.pGridGoal, &grid_map, &viPath);

	mapReader.saveGrid(&grid_map, sGridResLoc + sMapName + ".txt");

	//return 0;

	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobot robot;
	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	// Always try to connect to the first laser:
	argParser.addDefaultArgument("-connectLaser");

	if(!robotConnector.connectRobot())

	{
	ArLog::log(ArLog::Terse, "Could not connect to the robot.");
	if(argParser.checkHelpAndWarnUnparsed())
	{
		// -help not given, just exit.
		Aria::logOptions();
		Aria::exit(1);
	}
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
	Aria::logOptions();
	Aria::exit(1);
	}

	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);

	puts("Press  Escape to exit.");
  
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	robot.runAsync(true);

  
	// try to connect to laser. if fail, warn but continue, using sonar only
	if(!laserConnector.connectLasers())
	{
	ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
	}


	// turn on the motors
	robot.enableMotors();
 
	// add a set of actions that combine together to effect the wander behavior
	ArActionStallRecover recover;
	ArActionBumpers bumpers;
	FSM wanderAndAvoid;
	Wander wander;
	Avoid avoid;
	follow follow;

	FollowPath followPath;
	followPath.setPath(&viPath, &grid_map);

	///////////	ARIA	///////////
	robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	//robot.addAction(&avoid, 50);
	robot.addAction(&followPath, 40);
	//robot.addAction(&follow, 40);
	//robot.addAction(&wander, 1);

	
  
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();
  
	Aria::exit(0);
}
