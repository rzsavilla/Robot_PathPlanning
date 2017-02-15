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

	//Flip y axis so origin (0,0) is top left
	start.y = abs(start.y - m_pMapSize.y);
	end.y = abs(end.y - m_pMapSize.y);

	//start.x /= m_iCellSize;
	//start.y /= m_iCellSize;
	//end.x /= m_iCellSize;
	//end.y /= m_iCellSize;

	

	//Calculate distance from start to end
	Point fDiff;
	if (start.x >= end.x) fDiff.x = start.x - end.x;
	else fDiff.x = end.x - start.x;

	if (start.y >= end.y) fDiff.y = start.y - end.y;
	else fDiff.y = end.y - start.y;

	float fDistance = sqrt(pow(fDiff.x, 2) + pow(fDiff.y, 2));
	float fDistIncrement = m_iCellSize / 2;
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

MapReader::MapReader()
{

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

	for (int y = 0; y < grid->uiHeight; y++) {
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
			default:
				file << " ";
				break;
			}
		}
		file << "\n";
	}
	file.close();
}

bool MapReader::createGrid(std::string filename, Grid * grid, int cellSize)
{
	std::cout << "Parsing map file: " << filename << "\n";

	m_grid = grid;
	m_iCellSize = cellSize;	//Cell size in mm width and height

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

	int i[4];

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
				sscanf_s(line, "%*s %s %i %i %i", s, 20, &i[0], &i[1], &i[2]);
				if (!(strcmp(s, "Goal"))) {
					m_pGoalPos = Point(i[0],i[1]);
				}
				else if (!(strcmp(s, "RobotHome"))) {
					m_pStartPos = Point(i[0], i[1]);;
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
	m_pMapSize.x = abs(iMaxWidht) + abs(iMinWidht);		//Map width in mm
	m_pMapSize.y = abs(iMaxHeight) + abs(iMinHeight);		//Map height in mm

	//---------Create Grid----------------
	//Overlay a grid onto the map

	grid->uiWidth = ceil((float)m_pMapSize.x / m_iCellSize);
	grid->uiHeight = ceil((float)m_pMapSize.y / m_iCellSize);

	//Initialize grid
	std::cout << "Initializing grid: Width: " << grid->uiWidth << " Height: " << grid->uiHeight << "\n";
	grid->vNodes.clear();
	for (int y = 0; y < grid->uiHeight; y++) {
		for (int x = 0; x < grid->uiWidth; x++) {
			//Place node into grid (Index, State, Coordinate, ParentPointer)
			//Initialize grid setting all nodes as traversable '0'
			grid->vNodes.push_back(std::make_shared<Node>(getIndex(x, y, grid->uiWidth), 0, Point(x, y)));
		}
	}

	
	//Applied to points to ensure coordinates start from 0,0 top left corner of the grid
	Point pOffset = Point(abs(iMinWidht), abs(iMinHeight));

	// Set position to start at 0
	m_pStartPos.x += pOffset.x;
	m_pStartPos.y += pOffset.y;
	m_pGoalPos.x += pOffset.x;
	m_pGoalPos.y += pOffset.y;

	//Flip y coordinates
	m_pStartPos.y = abs(m_pStartPos.y - m_pMapSize.y);
	m_pGoalPos.y = abs(m_pGoalPos.y - m_pMapSize.y);

	//Set as grid coordinates
	m_pStartPos.x = (floor(m_pStartPos.x / m_iCellSize));
	m_pStartPos.y = (floor(m_pStartPos.y / m_iCellSize));
	m_pGoalPos.x = (floor(m_pGoalPos.x / m_iCellSize));
	m_pGoalPos.y = (floor(m_pGoalPos.y / m_iCellSize));

	//Place lines into grid map
	for (int i = 0; i < vLines.size(); i++) {
		vLines.at(i).start.x += pOffset.x;
		vLines.at(i).start.y += pOffset.y;
		vLines.at(i).end.x += pOffset.x;
		vLines.at(i).end.y += pOffset.y;

		placeLine(vLines.at(i).start, vLines.at(i).end);
	}

	return false;
}