#include "Aria.h"
#include "MapReader.h"
#include "AStar.h"
#include "FollowPath.h"

int main(int argc, char **argv)
{
	//////////	Generate Grid Map	///////////
	int iPadding = 2;
	int iCellSize = 150;

	MapReader mapReader(iCellSize, iPadding);
	std::string sMapResLoc = "resources/maps/";
	std::string sGridResLoc = "resources/grids/";
	std::string sMapName = "Mine";

	Grid grid_map;
	std::cout << "Generating Grid for: " << sMapName << "\n";
	std::cout << "CellSize: " << iCellSize << " Padding: " << iPadding << "\n";
	std::string sMapFile = (sMapResLoc.append(sMapName)).append(".map");
	mapReader.createGrid(sMapFile, &grid_map);
	
	////////// Initial A* Path Planning /////////////////
	//Initial path plan for given start and goal position in map file
	std::vector<int> viPath;	//Stores node indexes
	AStar pathFinder;
	pathFinder.addTraversable(1, 0);			//Traversable node states 0
	pathFinder.setMovementCost(1.0f, 2.0f);

	pathFinder.generatePath(grid_map.pMapStart, grid_map.pMapGoal, &grid_map, &viPath);

	mapReader.saveGrid(&grid_map, sGridResLoc + sMapName + ".txt");	//Save the grid to a text file

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

	FollowPath followPath;

	///////////	ARIA	///////////
	robot.addAction(&recover, 100);
	robot.addAction(&bumpers, 75);
	robot.addAction(&followPath, 40);

	//Set a path that follows maps start and goal posistions and the grid for path planning
	followPath.setPath(&viPath, &grid_map);
	
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();
  
	Aria::exit(0);
}
