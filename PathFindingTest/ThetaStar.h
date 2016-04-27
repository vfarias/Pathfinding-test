#pragma once
#include "AStar.h"
/*
Basic A* algorithm operating on a map with varying tile cost.
Should be relatively easy to expand for more specialized behavior
--Victor
*/
class ThetaStar : public AStar
{
private:
	void calculateGCost(Vec2D parentPos, Vec2D currentPos);
	bool lineOfSightBresenham(Vec2D parentPos, Vec2D currentPos);
	bool lineOfSightRay(Vec2D parentPos, Vec2D currentPos);
public:
	ThetaStar();
	ThetaStar(int width, int height, AStarNode** grid, Vec2D start, Vec2D goal, Heuristic heuristic = MANHATTAN);
	ThetaStar(int width, int height, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	virtual ~ThetaStar();
	bool findPath(Metrics& metrics);
};
