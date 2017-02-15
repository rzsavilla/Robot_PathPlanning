#pragma once

#include <memory>

struct Point {
	Point() {};
	Point(float x, float y) { this->x = x, this->y = y; };
	float x;
	float y;

	friend bool operator==(const Point& p1, const Point& p2) {
		if (p1.x == p2.x && p1.y == p2.y) return true;
	};
};

struct Node {
	Node() {};
	Node(int index, std::shared_ptr<Node> parent) {
		m_iIndex = index;
		m_ptrParent = parent;
	};
	Node(int index, int state, Point coordinate) {
		m_iIndex = index;
		m_iState = state;
		m_pCoord = coordinate;
	}
	int m_iIndex;
	int m_iState;
	float m_fScore;
	Point m_pCoord;
	std::shared_ptr<Node> m_ptrParent;
};