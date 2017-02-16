#pragma once

#include <string>
#include <vector>
#include <iostream>
#include "Node.h"
#include "Grid.h"

//struct Cell {
//	Cell() {};
//	Cell(int index, Cell* parent) { 
//		this->index = index;
//		ptrParent = parent;
//	};
//	int index;
//	int iState;
//	float fScore;
//	Cell* ptrParent;
//};

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

class MapReader {
private:
	Grid* m_grid;
	void placeLine(Point start, Point end);
	int addPadding(std::shared_ptr<Node> node, int MNeighbourhood);


	Point m_pMapSize;

	int m_iCellSize;
	int m_iPadding;

	float m_fMoveCost;	//Horizontal and vertical movement cost
	float m_fDMoveCost;	//Diagonal Movement cost
public:
	MapReader(int cellSize,int padding);
	void saveGrid(Grid* grid, std::string file);
	bool createGrid(std::string filename, Grid* grid);

	
};