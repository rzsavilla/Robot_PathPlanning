#pragma once

#include <vector>
#include <iostream>
#include "Node.h"

//Return index 1D based on coordinates
static int getIndex(int x, int y, int width) {
	return x + (y * width);
}

//Return grid coordinates based on index
static Point getCoord(int index, int width) {
	float x = (int)(index / width);	//Row
	float y = index % width;	//Column
	return Point(x, y);
}

//2D grid stores vector of nodes/cells
struct Grid {
	Grid() {};
	unsigned int uiWidth;	//!< Grid width;
	unsigned int uiHeight;	//!< Grid height;
	int iCellSize;
	Point pOffset;			//!< Used to change maps origin to 0,0
	Point pMapStart;		//!< Start position in map coordinates
	Point pMapGoal;			//!< Goal position in map coordinates
	Point pGridStart;		//!< Start position in grid coordinates
	Point pGridGoal;		//!< Goal position in grid coordinates
	float fStartTh;			//!< Robots initial heading
	std::vector<std::shared_ptr<Node>> vNodes;	//!< Grid cells/nodes
};
