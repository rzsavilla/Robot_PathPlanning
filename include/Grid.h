#pragma once

#include <vector>
#include <iostream>
#include "Node.h"

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
	std::vector<std::shared_ptr<Node>> vNodes;

	void draw() {
		for (int y = 0; y < uiHeight; y++) {
			for (int x = 0; x < uiWidth; x++) {
				int index = getIndex(x, y, this->uiWidth);
				switch (vNodes.at(index)->m_iState)
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
