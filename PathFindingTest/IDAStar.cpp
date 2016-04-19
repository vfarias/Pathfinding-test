

#include "IDAStar.h"

/*
Check to see if the pathfinding has reached its goal
*/
bool IDAStar::isAtGoal()
{
	return false;
}

void IDAStar::calculateHCost(Vec2D pos)
{

}
void IDAStar::calculateGCost(Vec2D parentPos, Vec2D currentPos)
{

}

IDAStar::IDAStar() :
	Pathfinding()
{
	_g = 0;
	_f = 0;

	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = 0;
	_height = 0;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = MANHATTAN;
}
IDAStar::IDAStar(int width, int height, Heuristic heuristicType) :
	Pathfinding(width, height, heuristicType)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = { 0, 0 };
	_goal = { 0, 0 };
	_heuristicType = heuristicType;
}
IDAStar::IDAStar(int width, int height, Vec2D start, Vec2D goal, Heuristic heuristicType) :
	Pathfinding(width, height, start, goal, heuristicType)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = start;
	_goal = goal;
	_heuristicType = heuristicType;
}

IDAStar::~IDAStar()
{
	delete[] _path;
	_path = nullptr;
}

Vec2D IDAStar::evaluateNode()
{
	return{ 0,0 };
}
bool findPath(Metrics& metrics)
{
	return false;
}