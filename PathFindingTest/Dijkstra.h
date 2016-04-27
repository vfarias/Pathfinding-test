#pragma once
#pragma once
#include "Pathfinding.h"
/*
	Dijkstra's algorithm is here used as a special case of A* where the heuristic is not considered
	The heuristic as a variable is still left in as it determines the type of movement allowed
*/
class Dijkstra : public Pathfinding
{
protected:
	Heap<AStarNode*> _openQueue;
	void calculateGCost(Vec2D parentPos, Vec2D currentPos);
public:
	Dijkstra();
	Dijkstra(int width, int height, Vec2D start, Vec2D goal, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	Dijkstra(int width, int height, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	virtual ~Dijkstra();
	void cleanMap();
	bool findPath(Metrics& metrics);
	float getPathLength();
};