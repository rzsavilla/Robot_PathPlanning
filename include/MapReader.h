#pragma once

#include <string>
#include <vector>
#include <iostream>
#include "Node.h"
#include "Grid.h"

//Represents data to create a line a start and an end
struct Line {
	Line(Point start, Point end) {
		this->start = start;
		this->end = end;
	}
	Line(int x1, int y1, int x2, int y2) {
		this->start.x = x1;
		this->start.y = y1;
		this->end.x = x2;
		this->end.y = y2;
	}

	Point start;
	Point end;
};

/**
* Generates a 2D grid using data from a Mapper3 map file.
* Essentially a grid overlay
*/
class MapReader {
private:
	Grid* m_grid;
	void placeLine(Point start, Point end);	//!< Mark grid node as untraversable if a line is within that cell

	//! Padding radius of extra untraversable nodes around an obstacle
	int addPadding(std::shared_ptr<Node> node, int MNeighbourhood);

	Point m_pMapSize;	//!< Width and height of the map

	int m_iCellSize;	//!< Size of each cell/node in the grid
	int m_iPadding;		//!< Radius of untraversable cells around an obstacle

	float m_fMoveCost;	//Horizontal and vertical movement cost
	float m_fDMoveCost;	//Diagonal Movement cost
public:
	MapReader(int cellSize,int padding);				//!< Constructor
	void saveGrid(Grid* grid, std::string file);		//!< Save grid as a text file
	bool createGrid(std::string filename, Grid* grid);	//!< Create grid by loading map file
};