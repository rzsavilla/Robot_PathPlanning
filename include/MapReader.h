#pragma once

#include <string>
#include <vector>
#include <iostream>



struct Cell {
	Cell() {};
	Cell(int index, Cell* parent) { 
		this->index = index;
		ptrParent = parent;
	};
	int index;
	int iState;
	Cell* ptrParent;
};

struct Point {
	Point() {};
	Point(int x, int y) { this->x = x, this->y = y; };
	int x;
	int y;
};

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

static int getIndex(int x, int y, int width) {
	return x + (y * width);
}

static Point getCoord(int index, int width) {
	int x = (int)(index / width);	//Row
	int y = index % width;	//Column
	return Point(x, y);
}

struct Grid {
	Grid() {};
	unsigned int uiWidth;
	unsigned int uiHeight;
	std::vector<Cell> vCells;

	void draw() {
		for (int y = 0; y < uiHeight; y++) {
			for (int x = 0; x < uiWidth; x++) {
				int index = getIndex(x, y, this->uiWidth);
				switch (vCells.at(index).iState)
				{
				case 0:
					std::cout << "-";		//Traversable
					break;
				case 1:
					std::cout << "#";		//Obstacle
					break;
				default:
					std::cout << "?";		//Unknown
					break;
				}
			}
			std::cout << "\n";
		}
	}
};

class MapReader {
private:
	void placeLine(Point start, Point end, Grid* grid);
	void placeLine(Point start, Point end, Grid* grid, Point minOffset);
	void placeLine(Line line, Grid* grid, Point minOffset);
public:
	MapReader();
	void saveGrid(Grid* grid, std::string file);
	bool readIntoGrid(std::string filename, Grid* grid);
};