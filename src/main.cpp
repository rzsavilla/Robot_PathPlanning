#include "Aria.h"
#include "wanderAndAvoid.h"
#include "wander.h"
#include "avoid.h"
#include "follow.h"
#include "MapReader.h"
#include "AStar.h"
#include "FollowPath.h"

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
	//pathFinder.setMovementCost(6.0f, 12.0f);
	pathFinder.setMovementCost(5.0f, 10.0f);
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
