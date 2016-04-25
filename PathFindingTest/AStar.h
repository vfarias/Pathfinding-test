#pragma once
#include "Pathfinding.h"
/*
Basic A* algorithm operating on a map with varying tile cost.
Should be relatively easy to expand for more specialized behavior
--Victor
*/
class AStar : public Pathfinding
{
protected:
	AStarNode** _grid;
	//std::vector<Node> _openQueue;							//A priority queue for open nodes
	Heap<AStarNode*> _openQueue;
	void calculateHCost(Vec2D pos);
	void calculateGCost(Vec2D parentPos, Vec2D currentPos);
public:
	AStar();
	AStar(int width, int height, Vec2D position, Vec2D start, Vec2D goal, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	AStar(int width, int height, Vec2D position, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	virtual ~AStar();
	void setTraversable(Vec2D pos, bool isTraversable = true);
	bool isTraversable(Vec2D pos)const;
	void cleanMap();
	bool findPath(Metrics& metrics);
	float getPathLength();
};