#pragma once

/*
A* Algorithm Tutorial
www.policyalmanac.org/games/aStarTutorial.htm
http://www.growingwiththeweb.com/2012/06/a-pathfinding-algorithm.html
*/

#pragma once

#include <vector>
#include <stdarg.h>     /* va_vector, va_start, va_arg, va_end */
#include <algorithm>
#include<memory>
#include "MapReader.h"
#include "Node.h"
#include "Grid.h"

class AStar {
private:
	Grid* m_grid;		//!< Pointer to 2D grid
	int m_iRootIndex;
	int m_iGoalIndex;
	Point m_pStart;
	Point m_pGoal;

	std::vector<unsigned int> m_vuiClosed;		//!< Visited Nodes
	std::vector<unsigned int> m_vuiOpen;		//!< Unvisited Nodes

	std::vector<unsigned int> m_vuiTraversable;		//!< Stores Node states that are traversable
private:
	std::vector<int> AStar::getAdjacent(unsigned int index);	//!< Returns indexes of adjacent nodes

	bool isTraversable(unsigned int index);		//!< Returns true if indexed node in grid is traversable

	bool isInOpen(unsigned int index);				//!< Check if node is in the open list
	bool isInClosed(unsigned int index);			//!< Check if node is in closed list
	float calcScore(std::shared_ptr<Node> node);			//!< Calculate nodes score
	bool savePath(unsigned int index, std::vector<int>* path);	//!< Store save path found by traversing through child node parent pointers 

	unsigned int getLowestScore(); //!< Returns index of node with the highest score within the open list
public:
	AStar();		//!< Default constructor

	void addTraversable(unsigned int n, ...);	//!< Add n number of traversable states
	std::vector<unsigned int> getTraversable();

	bool getPath(Point start, Point goal, Grid* grid, std::vector<int>* path);
};