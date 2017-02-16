#include "..\include\AStar.h"

std::vector<int> AStar::getAdjacent(unsigned int index)
{
	//Moore neighborhood with radius of 1
	std::vector<int> viAdjacentNodes;		//Stores adjacent/child nodes of center/parent node

	std::shared_ptr<Node> center = m_grid->vNodes.at(index);	//Center node
	Point p = center->m_pGridCoord;		//Coordinates of center node		
	unsigned int uiWidth = m_grid->uiWidth;		//Grid width used in computing index

	int iAdj;	//Index of adjacent node

	//Flags to determine if center node has these neighbours/bounds
	bool bTop = false;
	bool bBot = false;
	bool bLeft = false;
	bool bRight = false;

	//Check for bounds
	if (p.y > 0) bTop = true;						//Has top neighbour
	if (p.y < m_grid->uiHeight - 1) bBot = true;	//Has bot neighbour
	if (p.x > 0) bLeft = true;						//Has left neighbour
	if (p.x < m_grid->uiWidth - 1) bRight = true;	//Has right neighbour

	//Add adjacent/Neighbouring cells that are traversable
	if (bTop) {
		iAdj = getIndex(p.x, p.y - 1, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}
	if (bBot) {
		iAdj = getIndex(p.x, p.y + 1, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}
	if (bLeft) {
		iAdj = getIndex(p.x - 1, p.y, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}
	if (bRight) {
		iAdj = getIndex(p.x + 1, p.y, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}

	if (bTop && bLeft) {
		iAdj = getIndex(p.x - 1, p.y -1, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}
	if (bTop && bRight) {
		iAdj = getIndex(p.x + 1, p.y -1, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}
	if (bBot && bLeft) {
		iAdj = getIndex(p.x - 1, p.y + 1, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}
	if (bBot && bRight) {
		iAdj = getIndex(p.x + 1, p.y + 1, uiWidth);
		if (isTraversable(iAdj))	
			viAdjacentNodes.push_back(iAdj);	//Add child node
	}

	//Set Nodes parent
	for (auto it = viAdjacentNodes.begin(); it != viAdjacentNodes.end(); ++it) {
		m_grid->vNodes.at((*it))->m_ptrParent = m_grid->vNodes.at(index);	//Set parent
		calcScore(m_grid->vNodes.at((*it)));								//Calculate score
	}

	return viAdjacentNodes;
}

bool AStar::isTraversable(unsigned int index)
{
	int iState = m_grid->vNodes.at(index)->m_iState;	//Indexed node state
	//Iterate through traversable states compare with indexed nodes current state.
	for (auto it = m_vuiTraversable.begin(); it != m_vuiTraversable.end(); ++it) {
		if ((*it) == iState) {			//Node state is traversable/Not an obstacle
			if (!isInClosed(index)) {	//Not in the closed list
				if (!isInOpen(index)) {	//In the open list
					return true;
				}
			}
		}
	}
	return false;
}

bool AStar::isInOpen(unsigned int index)
{
	//Search through open list
	for (auto it = m_vuiOpen.begin(); it != m_vuiOpen.end(); ++it) {
		if ((*it) == index) return true;
	}
	return false;
}

bool AStar::isInClosed(unsigned int index)
{
	//Search through closed list
	for (auto it = m_vuiClosed.begin(); it != m_vuiClosed.end(); ++it) {
		if ((*it) == index) return true;
	}
	return false;
}

float AStar::calcScore(std::shared_ptr<Node> node)
{
	/*	Score = G + H
	G = Movement cost from starting point
	H = Estimated movement cost from this node to destination node (Heuristic)
	*/
	if (node->m_ptrParent != NULL) {	//Check if it has a parent
		int MoveCost = 10;

		float G = 0.0f;
		float H = 0.0f;

		int iMoveTotal = 0;
		int iChild;
		int iParent;
		iChild = node->m_iIndex;
		iParent = node->m_ptrParent->m_iIndex;

		//Flag determines if root node has been reached
		bool bRootReached = (m_grid->vNodes.at(iChild)->m_iIndex == m_iRootIndex);
		std::shared_ptr<Node> ptrParent = node->m_ptrParent;
		
		//Calculate G: Movement cost from root node to current child node
		
		float fDMoveCost = 15.0f;
		float fMoveCost = 10.0f;

		float fCost;
		fCost = fMoveCost;
		if (node->m_ptrParent->m_pGridCoord.y != node->m_pGridCoord.y) {
			if (node->m_ptrParent->m_pGridCoord.x != node->m_pGridCoord.x) {
				//Diagonal movement cost
				fCost = fMoveCost = fDMoveCost;
			}
		}

		node->m_fScoreG = node->m_ptrParent->m_fScoreG + fCost;
		
		/*
		while (!bRootReached) {
			Point pChild = m_grid->vNodes.at(iChild)->m_pGridCoord;		//Get child node coordinates
			Point pParent = m_grid->vNodes.at(iParent)->m_pGridCoord;	//Get parent node coordinates
																	//Move direction from parent to child position
			Point dir = Point(pChild.x - pParent.x, pChild.y - pParent.y);
			//Horizontal move
			if (dir.x > 0 || dir.x < 0) {
				G += MoveCost;
			}
			//Vertical move
			else if (dir.y > 0 || dir.y < 0) {
				G += MoveCost;
			}
			else {
				G += MoveCost;
			}
			iMoveTotal++;

			iChild = ptrParent->m_iIndex;	//Child's parent is now the child node

			//Check if root node reached
			bRootReached = (m_grid->vNodes.at(iChild)->m_iIndex == m_iRootIndex);
			if (bRootReached) continue;	//Skip

			ptrParent = ptrParent->m_ptrParent;	//Get parents parent
			iParent = ptrParent->m_iIndex;			//Previous parent nodes parent is now the parent
		}
		node->m_fScoreG = G;
		*/
		/*
		Heuristics
		http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
		*/
		
		//This nodes grid coordinates
		Point pCoordCurrent = node->m_pGridCoord;		
		float dx = abs(pCoordCurrent.x - m_pGoal.x);
		float dy = abs(pCoordCurrent.y - m_pGoal.y);
		//H = (1 * sqrt(dx * dx + dy * dy)) * 10;

		H = fMoveCost * (dx + dy) + (fDMoveCost - 2 * fMoveCost) * std::min(dx, dy);

		float fAnswer = node->m_fScoreG + H;
		node->m_fScore = fAnswer;
		return fAnswer;
	}
	return 0.0f;
}

bool AStar::savePath(unsigned int index, std::vector<int>* path)
{
	//Path from child node all the way to the root node/goal node
	path->clear();	//Ensure vector is empty
	std::shared_ptr<Node> node = m_grid->vNodes.at(index);
	while (node->m_ptrParent != nullptr) {
		node->m_iState = 3;
		path->push_back(node->m_iIndex);
		node = node->m_ptrParent;
	}
	std::reverse(path->begin(), path->end());	//First element becomes starting node last is goal
	return true;
}

unsigned int AStar::getLowestScore()
{
	float fLowest = 99999999.0f;
	int iIndex = 0;	//Index of node with highest score
	for (int i = 0; i < m_vuiOpen.size(); i++) {
		float fScore = m_grid->vNodes.at(m_vuiOpen.at(i))->m_fScore;
		if (fScore < fLowest) {
			fLowest = fScore;
			iIndex = m_vuiOpen.at(i);
		}
	}
	return iIndex;
}

AStar::AStar()
{
}

void AStar::addTraversable(unsigned int n, ...)
{
	unsigned int val;
	va_list vl;
	va_start(vl, n);
	for (unsigned int i = 0; i < n; i++) {
		val = va_arg(vl, int);
		m_vuiTraversable.push_back(val);
	}
}

std::vector<unsigned int> AStar::getTraversable()
{
	return m_vuiTraversable;
}

bool AStar::getPath(Point start, Point goal, Grid * grid, std::vector<int>* path)
{
	std::cout << "Finding Path\n ";
	std::cout << "Start:" << start.x << " " << start.y << " Goal: " << goal.x << " " << goal.y << "\n";
	//std::cout << "Get Path\n";
	if (!grid->vNodes.empty()) {
		m_vuiOpen.clear();
		m_vuiClosed.clear();

		m_grid = grid;
		m_iRootIndex = getIndex(start.x,start.y,grid->uiWidth);
		m_iGoalIndex = getIndex(goal.x, goal.y, grid->uiWidth);
		m_pStart = m_grid->vNodes.at(m_iRootIndex)->m_pGridCoord;
		m_pGoal = m_grid->vNodes.at(m_iRootIndex)->m_pGridCoord;

		//Add adjacent nodes
		std::vector<int> vuiAdjacent;
		vuiAdjacent = getAdjacent(m_iRootIndex);
		//Add adjacent nodes to open list
		m_vuiOpen.insert(m_vuiOpen.end(), vuiAdjacent.cbegin(), vuiAdjacent.cend());
		vuiAdjacent.clear();
		m_vuiClosed.push_back(m_iRootIndex);	//Add Root node to closed list

		//Traverse grid to find shortest path to goal node
		while (!m_vuiOpen.empty()) {
			//Find the node with the lowest score
			int iChosenNode = getLowestScore();	//Node with the lowest score
			//std::cout << m_vOpen.at(0)->getScore() << " " << m_vOpen.at(0)->getIndex() << "\n";

			//Add adjacent nodes
			vuiAdjacent = getAdjacent(iChosenNode);
			m_vuiOpen.insert(m_vuiOpen.end(), vuiAdjacent.cbegin(), vuiAdjacent.cend());	//Add Adjacent nodes to open list
			vuiAdjacent.clear();

			//Check if goal has been reached
			if (iChosenNode == m_iGoalIndex) {
				//Save path and end search
				savePath(iChosenNode, path);
				std::cout << "Path found: Nodes: " << path->size() << "\n";
				return true;
			}
			//Point coord = getCoord(iChosenNode,m_grid->uiWidth);
			//std::cout << coord.x << " " << coord.y << "\n";
			//std::cout << "OpenList: " << m_vuiOpen.size() << " ClosedList: " << m_vuiClosed.size() << "\n";
			//Remove node from open and add to closed list
			m_vuiClosed.push_back(iChosenNode);
			for (int i = 0; i < m_vuiOpen.size(); i++) {
				if (m_vuiOpen.at(i) == iChosenNode) {
					m_vuiOpen.erase(m_vuiOpen.begin() +i);	//Remove node 
					break;
				}
				
			}
		}
	}
	//No Path found
	std::cout << "No path found\n";
	return false;
}
