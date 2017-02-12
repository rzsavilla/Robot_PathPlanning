#include "MapReader.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

void MapReader::placeLine(Point start, Point end,Grid* grid)
{
	//Calculate distance from start to end
	Point pDist = Point(end.x - start.x, end.y - start.y);
	int distance = ceil(sqrt(pow(pDist.x, 2) + pow(pDist.y,2)));

	//Start point
	//int iIndex = getIndex(start.x, start.y, grid->uiWidth);

	Point pDrawDist = Point(0,0);	//Distance on the line to draw

	//Find point along the line at distance d
	int iD = 0;

	//http://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point

	float fMag;	//magnitude of distance vector
	fMag = sqrt((pDist.x * pDist.x) + (pDist.y * pDist.y));
	Point unitNormal = Point(pDist.x / fMag, pDist.y / fMag);

	float dX = 0;
	float dY = 0;
	int iTargetDist = 1;

	Point pStart;
	int iIndex;
	Point p;

	//Line draw 
	float uX, uY;

	bool bX = false;
	bool bY = false;
	bool b = false;
	std::cout << "Placing Line: x:" << start.x << " y:" << start.y << " x:" << end.x << " y:" << end.y << "\n";
	while (!b) {
		std::cout << "Placing\n";
		if (!bX) p.x = start.x + dX;
		if (!bY) p.y = start.y + dY;
		//std::cout << "x:" << p.x << " y:" << p.y << "\n"; 

		iIndex = getIndex(p.x, p.y, grid->uiWidth);
		if (iIndex >= grid->vCells.size()) {
			iTargetDist = distance;
			continue;
		}
		else {
			grid->vCells.at(iIndex).iState = 1;
		}

		uX = (pDist.x / fMag);
		uY = (pDist.y / fMag);
		if (uX > 0) uX = ceil(uX);	//Round up
		else uX = floor(uX);		//Round down
		if (uY > 0) uY = ceil(uY);
		else uY = floor(uY);

		dX = (iTargetDist * uX);
		dY = (iTargetDist * uY);
		if (dX > 0) dX = ceil(dX);
		else dX = floor(dX);
		if (dY > 0) dY = ceil(dY);
		else dY = floor(dY);

		iTargetDist++;

		bX = p.x == end.x;
		bY = p.y == end.y;
		b = bX && bY;

		//grid->draw();
		//std::cout << "\n";
	} 
	std::cout << "Line placed: x:" << start.x << " y:" << start.y << " x:" << end.x << " y:" << end.y << "\n";
	//grid->draw();
	//system("pause");
}

void MapReader::placeLine(Point start, Point end, Grid* grid, Point minOffset)
{
	Point newStart;
	Point newEnd;

	Point p = Point(abs(minOffset.x), abs(minOffset.y));

	newStart = Point((start.x + p.x), (start.y + p.y));
	newEnd = Point((end.x + p.x) , (end.y + p.y));

	placeLine(newStart,newEnd,grid);
}

void MapReader::placeLine(Line line, Grid* grid, Point minOffset)
{
	placeLine(line.start, line.end, grid,minOffset);
}

MapReader::MapReader()
{

}

void MapReader::saveGrid(Grid * grid, std::string filename)
{
	std::ofstream file;
	file.open(filename);
	if (!file.is_open()) {
		std::cout << "*Level could not be saved: '" << filename << "'\n\n";
		return;
	}

	for (int y = 0; y < grid->uiHeight; y++) {
		for (int x = 0; x < grid->uiWidth; x++) {
			int i = grid->vCells.at(getIndex(x, y, grid->uiWidth)).iState;
			switch (i)
			{
			case 0:
				file << " ";
				break;
			case 1:
				file << "#";
				break;
			default:
				file << " ";
				break;
			}
		}
		file << "\n";
	}
	file.close();
}

bool MapReader::readIntoGrid(std::string filename)
{
	FILE* myFile;

	std::string sFileLocation = filename;

	char* cFilename = new char[100];
	strcpy_s(cFilename, 100, sFileLocation.c_str());

	//Attributes
	int iMinWidht;
	int iMinHeight;
	int iMaxWidht;
	int iMaxHeight;

	Grid grid;

	//Open the file
	fopen_s(&myFile, cFilename, "r");

	std::vector<Line> vLines;	//Stores data required to draw lines

	char line[100];
	if (myFile != NULL) {
		while (fgets(line, 100, myFile)) {
			char caType[20] = " ";	//Empty

			sscanf_s(line, "%s ", caType, 20);
			if (!(strcmp(caType, "LineMinPos:"))) {
				sscanf_s(line, "%*s %i %i", &iMinWidht, &iMinHeight);
				std::cout << "Found\n";
			}
			else if (!(strcmp(caType, "LineMaxPos:"))) {
				sscanf_s(line, "%*s %i %i", &iMaxWidht, &iMaxHeight);
				std::cout << "Found\n";
			}
			else if (!(strcmp(caType, "LINES"))) {
				//Read lines
				sscanf_s(line, "%s ", caType, 20);
				while ((strcmp(caType, "DATA"))) {
					fgets(line, 100, myFile);
					sscanf_s(line, "%s ", caType, 20);
					if (!(strcmp(caType, "DATA"))) continue;
					//Read line data
					Line l(0, 0, 0, 0);
					sscanf_s(line, "%i %i %i %i", &l.start.x, &l.start.y, &l.end.x, &l.end.y);
					vLines.push_back(l);
				}
			}
		}
	}

	int iReduce = 13;

	int iPadding = 10;

	iMinWidht -= iPadding;
	iMinHeight -= iPadding;
	iMaxWidht += iPadding;
	iMaxHeight += iPadding;

	iMinWidht /= iReduce;
	iMinHeight /= iReduce;
	iMaxWidht /= iReduce;
	iMaxHeight /= iReduce;

	grid.uiWidth = abs(iMaxWidht) + abs(iMinWidht);
	grid.uiHeight = abs(iMaxHeight) + abs(iMinHeight);

	int iSize = (grid.uiWidth * grid.uiHeight);

	int iWidthOffset = iMinWidht;
	int iHeightOffset = iMinHeight;

	//Initialize grid
	Cell emptyCell;
	emptyCell.iState = 0;
	emptyCell.ptrParent = NULL;
	std::cout << "Filling grid\n";
	for (int i = 0; i < iSize; i++) {
		grid.vCells.push_back(emptyCell);
	}

	//vLines.push_back(Line(0, 0, 9, 9));
	//vLines.push_back(Line(0, 5, 9, 5));
	for (int i = 0; i < vLines.size(); i++) {
		vLines.at(i).start.x /= iReduce;
		vLines.at(i).start.y /= iReduce;
		vLines.at(i).end.x /= iReduce;
		vLines.at(i).end.y /= iReduce;
		placeLine(vLines.at(i), &grid, Point(iMinWidht, iMinHeight));
	}
	std::cout << "Saving to txt file";
	std::cout << "Saving grid map\n";
	saveGrid(&grid, "gridmap.txt");
	//grid.draw();
	return false;
}
