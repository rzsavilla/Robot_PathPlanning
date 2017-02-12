#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <Aria.h>


#include "odometry.h"
// Implementation

// Constructor
odometry::odometry() : ArAction("Odometry")
{
	myX = 0.0; // init x pos
	myY = 0.0; // init y pos
	myTh = 0.0; // init heading

	l = 425.0; // Width of a P3 DX in mm
	rw = 95.0; // Wheel radius in mm
	tr = 360.0; // Number of odemetry clicks for a full circle
	oldTs = GetTickCount();

	fDistTravelled = 0.0f;

	file.open("robot.csv");
}

odometry::~odometry()
{
	file.close();
}

// Body of action
ArActionDesired * odometry::fire(ArActionDesired d)
{
	desiredState.reset(); // reset the desired state (must be done)

						  // Get ARIA odometer readings 
	robotX = myRobot->getX();
	robotY = myRobot->getY();
	robotTh = myRobot->getTh(); // (in degrees)

	simulateEncoders(); // Sets t1 and t2, on the real robot these would be set by calling myRobot->getLeftEncoder();

	float Pi = 3.14159265359;
	double dist = 0;
	double dx = 0;
	double dy = 0;
	double dTh = 0;

	float radians = robotTh * Pi / 180;

	dx = cos(radians) * (t1 + t2) * ((Pi * rw) / tr);
	dy = sin(radians) * (t1 + t2) * ((Pi * rw) / tr);

	dTh = radians + (2 * Pi) * (t2 - t1) * (rw / (tr *l));

	myX = myX + dx;
	myY = myY + dy;
	myTh = myTh + dTh;

	fDistTravelled += abs(dx);
	fDistTravelled += abs(dy);

	//file << robotX << "," << robotY << "," << (robotTh / 57.3f) << "," << myX << "," << myY << "," << (robotTh / 57.3f) << "\n";

	std::cout << "Velocity: " << myRobot->getVel() << "\tHeading:" << myRobot->getTh() << "\n";
	std::cout << "Position: X:" << myRobot->getX() << " Y:" << myRobot->getY() << "\n";
	std::cout << "Distance travelled: " << fDistTravelled << "\n";
	//printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", robotX, robotY, (robotTh / 57.3f), myX, myY, (robotTh / 57.3f));

	return &desiredState; // give the desired state to the robot for actioning
}


void odometry::simulateEncoders()
{
	newTs = GetTickCount();

	long long unsigned int oldt = (long long unsigned int)oldTs;   // Last time click as 64bit 
	long long unsigned int newt = (long long unsigned int)newTs;  // New time click as 64bit 
	int diff = newt - oldt; // Time since last calculation

	double d1 = ((double)diff * myRobot->getLeftVel()) / 1000.0f; // D = speed * time / 1000000.0 to get into seconds
	double d2 = ((double)diff * myRobot->getRightVel()) / 1000.0f;

	t1 = floor((d1 * tr) / (2.0f * 3.14f * rw)); // Rearranged odmetry equation, round as ticks would always be ints
	t2 = floor((d2 * tr) / (2.0f * 3.14f * rw));

	oldTs = newTs; // Set old time
}

