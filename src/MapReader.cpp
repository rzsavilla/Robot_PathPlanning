#include "MapReader.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>

void MapReader::placeLine(Point start, Point end)
{
	/* 
	Looks at every point in the line and checks if it is inside a grid cell.
	If a point in the line is within the cell then the cell in untraversable given a value of 1

	Formula, finding point a long a line
	http://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
	*/

	//Calculate distance from start to end
	Point fDiff;
	if (start.x >= end.x) fDiff.x = start.x - end.x;
	else fDiff.x = end.x - start.x;

	if (start.y >= end.y) fDiff.y = start.y - end.y;
	else fDiff.y = end.y - start.y;

	float fDistance = sqrt(pow(fDiff.x, 2) + pow(fDiff.y, 2));
	float fDistIncrement = m_iCellSize / 4;
	float dt = 0;
	

	Point p;
	while (dt <= fDistance) {
		float f = dt/ fDistance;
		Point pLinePoint;
		if (start.x > end.x) 
			pLinePoint.x = (start.x - (f * fDiff.x));
		else 
			pLinePoint.x = (start.x + (f * fDiff.x));

		if (start.y > end.y) 
			pLinePoint.y = (start.y - (f * fDiff.y));
		else 
			pLinePoint.y = (start.y + (f * fDiff.y));

		//Point Coordinates into grid coordinates

		Point pGridCoord;
		pGridCoord.x = (floor(pLinePoint.x / m_iCellSize));
		pGridCoord.y = (floor(pLinePoint.y / m_iCellSize));
		
		int index = getIndex(pGridCoord.x, pGridCoord.y, m_grid->uiWidth);
		if (index >= m_grid->vNodes.size()) index = m_grid->vNodes.size() - 1;
		m_grid->vNodes.at(index)->m_iState = 1;

		dt+= fDistIncrement;
		//std::cout << pGridCoord.x << " " << pGridCoord.y << "\n";
	}
	std::cout << "Line placed: x:" << start.x << " y:" << start.y << " x:" << end.x << " y:" << end.y << "\n";
}

int MapReader::addPadding(std::shared_ptr<Node> node, int MNeighbourhood)
{
	//Count neighbours
	int RangeX = MNeighbourhood;	//Moore neighbourhood size
	int RangeY = MNeighbourhood;
	int x = node->m_pGridCoord.x;
	int y = node->m_pGridCoord.y;
	int startX = x - RangeX;
	int startY = y - RangeY;
	int endX = x + RangeX;
	int endY = y + RangeY;
	//Check if within map
	if (startX < 0) startX = 0;
	if (startY < 0) startY = 0;
	if (startX > m_grid->uiWidth - 1) startX = m_grid->uiWidth - 1;
	if (startY > m_grid->uiHeight - 1) startY = m_grid->uiHeight - 1;
	if (endX < 0) endX = 0;
	if (endY < 0) endY = 0;
	if (endX > m_grid->uiWidth - 1) endX = m_grid->uiWidth - 1;
	if (endY > m_grid->uiHeight - 1) endY = m_grid->uiHeight - 1;

	for (int y = startY; y <= endY; y++) {
		for (int x = startX; x <= endX; x++) {
			int index = getIndex(x, y, m_grid->uiWidth);
			if (m_grid->vNodes.at(index)->m_iState != 1) {
				m_grid->vNodes.at(index)->m_iState = 2;	//Change to untraversable
			}
		}
	}
	return 0;
}


MapReader::MapReader(int cellSize, int padding)
{
	m_iCellSize = cellSize;
	m_iPadding = padding;
}

void MapReader::saveGrid(Grid * grid, std::string filename)
{
	std::cout << "Saving Grid Map: " << filename << "\n";
	std::ofstream file;
	file.open(filename);
	if (!file.is_open()) {
		std::cout << "*Level could not be saved: '" << filename << "'\n\n";
		return;
	}

	for (int y = grid->uiHeight-1; y >= 0; y--) {
		for (int x = 0; x < grid->uiWidth; x++) {
			int iIndex = getIndex(x, y, grid->uiWidth);
			int i = grid->vNodes.at(getIndex(x, y, grid->uiWidth))->m_iState;
			switch (i)
			{
			case 0:
				file << " ";
				break;
			case 1:
				file << "#";
				break;
			case 2:
				file << "@";
				break;
			case 3:
				file << "+";
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

bool MapReader::createGrid(std::string filename, Grid * grid)
{
	std::cout << "Parsing map file: " << filename << "\n";

	m_grid = grid;
	m_grid->iCellSize = m_iCellSize;

	//----------Parse map file-----------
	FILE* myFile;
	std::string sFileLocation = filename;
	char* cFilename = new char[100];
	strcpy_s(cFilename, 100, sFileLocation.c_str());

	//Attributes
	int iMinWidht;
	int iMinHeight;
	int iMaxWidht;
	int iMaxHeight;

	int i[4];	//Temp integer storage for parsing

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
			}
			else if (!(strcmp(caType, "LineMaxPos:"))) {
				sscanf_s(line, "%*s %i %i", &iMaxWidht, &iMaxHeight);
			}
			else if (!(strcmp(caType, "Cairn:"))) {
				char ca[100];
				Point p;
				float th;
				char s[20];
				sscanf_s(line, "%*s %s %i %i %f", s, 20, &i[0], &i[1], &th);
				if (!(strcmp(s, "Goal"))) {
					grid->pMapGoal = Point(i[0],i[1]);
					
				}
				else if (!(strcmp(s, "RobotHome"))) {
					grid->pMapStart = Point(i[0], i[1]);;
					grid->fStartTh = th;
				}
			}
			else if (!(strcmp(caType, "LINES"))) {
				//Read lines
				sscanf_s(line, "%s ", caType, 20);
				while ((strcmp(caType, "DATA"))) {
					fgets(line, 100, myFile);
					sscanf_s(line, "%s ", caType, 20);
					if (!(strcmp(caType, "DATA"))) continue;
					//Read line data
					sscanf_s(line, "%i %i %i %i", &i[0], &i[1], &i[2], &i[3]);
					Line l(i[0], i[1], i[2], i[3]);
					vLines.push_back(l);
				}
			}
		}
	}
	m_pMapSize.x = abs((iMaxWidht) - (iMinWidht));		//Map width in mm
	m_pMapSize.y = abs((iMaxHeight) - (iMinHeight));	//Map height in mm

	//---------Create Grid----------------
	//Overlay a grid onto the map

	grid->uiWidth = ceil((float)m_pMapSize.x / m_iCellSize);
	grid->uiHeight = ceil((float)m_pMapSize.y / m_iCellSize);

	//Initialize grid
	std::cout << "Initializing grid: Width: " << grid->uiWidth << " Height: " << grid->uiHeight << "\n";
	grid->vNodes.clear();
	for (int y = 0; y < grid->uiHeight; y++) {
		for (int x = 0; x < grid->uiWidth; x++) {
			//Place node into grid (Index, State, GridCoordinate, MapCoordinate)
			//Initialize grid setting all nodes as traversable '0'
			grid->vNodes.push_back
			(
				std::make_shared<Node>(
					getIndex(x, y, grid->uiWidth),		// Index
					0,									// State
					Point(x,y),							// Grid Coordinate
					Point((x * m_iCellSize) + m_iCellSize / 2,	//Map coordinate
					(y * m_iCellSize) + m_iCellSize / 2)
				)
			);
		}
	}

	//Applied to points to ensure coordinates start from 0,0 top left corner of the grid
	Point pOffset = Point(abs(iMinWidht), abs(iMinHeight));
	grid->pOffset = pOffset;

	// Set position to start at 0,0
	grid->pMapStart.x += pOffset.x;
	grid->pMapStart.y += pOffset.y;
	grid->pMapGoal.x += pOffset.x;
	grid->pMapGoal.y += pOffset.y;

	//Convert map coordinates into grid coordinates
	grid->pGridStart.x = (floor(grid->pMapStart.x / m_iCellSize));
	grid->pGridStart.y = (floor(grid->pMapStart.y / m_iCellSize));
	grid->pGridGoal.x = (floor(grid->pMapGoal.x / m_iCellSize));
	grid->pGridGoal.y = (floor(grid->pMapGoal.y / m_iCellSize));

	//Place lines into grid map
	for (int i = 0; i < vLines.size(); i++) {
		vLines.at(i).start.x += pOffset.x;	//Apply offset to set origin to 0,0
		vLines.at(i).start.y += pOffset.y;
		vLines.at(i).end.x += pOffset.x;
		vLines.at(i).end.y += pOffset.y;
		placeLine(vLines.at(i).start, vLines.at(i).end);
	}

	//Add Padding/Neighbour cells become untraversable
	for (int i = 0; i < m_grid->vNodes.size(); i++) {
		if (m_grid->vNodes.at(i)->m_iState == 1) {
			addPadding(m_grid->vNodes.at(i),m_iPadding);
		}
	}

	return true;
}