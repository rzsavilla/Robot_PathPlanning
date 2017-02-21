#include "AStar.h"

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
		if (m_grid->vNodes.at((*it))->m_ptrParent == NULL) {
			m_grid->vNodes.at((*it))->m_ptrParent = m_grid->vNodes.at(index);	//Set parent
		}
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
				if (!isInOpen(index)) {	//Not the open list
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
		float fCost;
		fCost = m_fMoveCost;
		if (node->m_ptrParent->m_pGridCoord.y != node->m_pGridCoord.y) {
			if (node->m_ptrParent->m_pGridCoord.x != node->m_pGridCoord.x) {
				//Diagonal movement cost
				fCost = m_fDMoveCost;
			}
		}

		node->m_fScoreG = node->m_ptrParent->m_fScoreG + fCost;
		/*
		Heuristics Forumula
		http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
		*/
		//This nodes grid coordinates
		Point pCoordCurrent = node->m_pGridCoord;		
		float dx = abs(pCoordCurrent.x - m_pGoal.x);
		float dy = abs(pCoordCurrent.y - m_pGoal.y);

		H = m_fMoveCost * (dx + dy) + (m_fDMoveCost - 2 * m_fMoveCost) * std::min(dx, dy);

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
	std::cout << "Saving Path\n";
	while (node->m_ptrParent != NULL && node->m_iIndex != m_iRootIndex) {
		node = node->m_ptrParent;
		path->push_back(node->m_iIndex);
		
		node->m_iState = 3;	//Node state is now a path
	}

	////Reset node parent pointers
	for (auto it = m_grid->vNodes.begin(); it != m_grid->vNodes.end(); ++it) {
		if ((*it)->m_ptrParent != NULL) {
			(*it)->m_ptrParent = NULL;
		}
	}
	std::reverse(path->begin(), path->end());	//First element becomes starting node last is goal
	path->erase(path->begin());	//Remove starting node/node robot is currently on
	
	
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
	m_fDMoveCost = 0.0f;
	m_fMoveCost = 0.0f;
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

void AStar::setMovementCost(float straight, float diagonal)
{
	m_fMoveCost = straight;
	m_fDMoveCost = diagonal;
}

std::vector<unsigned int> AStar::getTraversable()
{
	return m_vuiTraversable;
}

bool AStar::generatePath(Point start, Point goal, Grid * grid, std::vector<int>* path)
{
	std::cout << "Finding Path\n ";
	std::cout << "Start:" << start.x << " " << start.y << " Goal: " << goal.x << " " << goal.y << "\n";
	//std::cout << "Get Path\n";
	
	if (!grid->vNodes.empty()) {
		m_vuiOpen.clear();
		m_vuiClosed.clear();
		path->clear();
		m_grid = grid;

		//Convert map coordinates into grid coordinates
		m_pStart.x = floor(start.x / grid->iCellSize);
		m_pStart.y = floor(start.y / grid->iCellSize);
		m_pGoal.x = floor(goal.x / grid->iCellSize);
		m_pGoal.y = floor(goal.y / grid->iCellSize);

		//Get grid index
		m_iRootIndex = getIndex(m_pStart.x, m_pStart.y, grid->uiWidth);
		m_iGoalIndex = getIndex(m_pGoal.x, m_pGoal.y, grid->uiWidth);
		
		//Add root node
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

			//Remove node from open and add to closed list
			m_vuiClosed.push_back(iChosenNode);
			for (int i = 0; i < m_vuiOpen.size(); i++) {
				if (m_vuiOpen.at(i) == iChosenNode) {
					m_vuiOpen.erase(m_vuiOpen.begin() + i);	//Remove node 
					break;
				}

			}
		}
	}
	//std::cout << "No path found\n";
	return false;

}
