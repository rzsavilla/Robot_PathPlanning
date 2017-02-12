// Signatures
#include <fstream>

class odometry : public ArAction // Class action inherits from ArAction
{
public:
	odometry(); // Constructor
	virtual ~odometry();  // Destructor
	virtual ArActionDesired * fire(ArActionDesired d); // Body of the action
	ArActionDesired desiredState; // Holds state of the robot that we wish to action

protected:
	double robotX, robotY, robotTh; // Robots position according to Aria
	double myX, myY, myTh; // Robots position according to your odometry calculations
	double t1, t2; // Wheel encoder counts
	double tr; // Radius of the wheel
	double l; // Wheelbase of the robot
	double rw; // Wheel radius of the robot

	float fDistTravelled;
	void simulateEncoders();
	DWORD newTs, oldTs;

	std::ofstream file;
};
