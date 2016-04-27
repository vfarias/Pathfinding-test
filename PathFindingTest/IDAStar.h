

#pragma once

#include "Pathfinding.h"

using namespace std;

class IDAStar : public Pathfinding
{
private:
	//BaseNode node;
	//float _g;
	//float _f;
	bool isAtGoal(/*Node start*/);
	void calculateHCost(Vec2D pos);
	void calculateGCost(Vec2D parentPos, Vec2D currentPos);
public:
	IDAStar();
	IDAStar(int width, int height, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	IDAStar(int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	~IDAStar();
	Vec2D evaluateNode( Vec2D Node, float g, float threshold);
	bool findPath(Metrics& metrics);
};