

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
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = 0;
	_height = 0;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = MANHATTAN;
	_grid = nullptr;
}
IDAStar::IDAStar(int width, int height, AStarNode** grid, Heuristic heuristicType) :
	Pathfinding(width, height, {0,0}, heuristicType)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = { 0, 0 };
	_goal = { 0, 0 };
	_heuristicType = heuristicType;
	_grid = grid;
	for (__int16 i = _position._x; i < _position._x + _width; i++)
	{
		for (__int16 j = _position._y; j < _position._y + _height; j++)
		{
			_grid[i][j]._open = 0;
			_grid[i][j]._gCost = 0;
			_grid[i][j]._hCost = 0;
			_grid[i][j]._parent = nullptr;
		}
	}
}
IDAStar::IDAStar(int width, int height, Vec2D start, Vec2D goal, AStarNode** grid, Heuristic heuristicType) :
	Pathfinding(width, height, start, goal, {0,0}, heuristicType)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = start;
	_goal = goal;
	_heuristicType = heuristicType;
	_grid = grid;
	for (__int16 i = _position._x; i < _position._x + _width; i++)
	{
		for (__int16 j = _position._y; j < _position._y + _height; j++)
		{
			_grid[i][j]._open = 0;
			_grid[i][j]._gCost = 0;
			_grid[i][j]._hCost = 0;
			_grid[i][j]._parent = nullptr;
		}
	}
}

IDAStar::~IDAStar()
{
}

Vec2D IDAStar::evaluateNode(Vec2D pos, float g, float threshold)
{
	_grid[pos._x][pos._y]._gCost = g;
	if (pos == _goal || (g + getHeuristicDistance(pos, _goal)) - threshold > 0.1f)
	{
		return pos;
	}
	Vec2D minPos = {pos._x - 1, pos._y};
	float minValue = -1.0f;
	for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
	{
		Vec2D checkedPos = pos + NEIGHBOUR_OFFSETS[i];
		_grid[checkedPos._x][checkedPos._y]._parent = &_grid[pos._x][pos._y];
		
		float tileDist = 1;
		if (i >= 4)
		{
			tileDist = SQRT2;
		}
		if (_grid[checkedPos._x][checkedPos._y]._traversable)
		{
			checkedPos = evaluateNode(checkedPos, g + tileDist, threshold);

			if (checkedPos == _goal)
			{
				return checkedPos;
			}

			float checkedValue = _grid[checkedPos._x][checkedPos._y]._gCost + getHeuristicDistance(checkedPos, _goal);
			if (checkedValue < minValue || minValue < 0)
			{
				minValue = checkedValue;
				minPos = checkedPos;
			}
		}
	}
	return minPos;
}

void IDAStar::setTraversable(Vec2D pos, bool isTraversable)
{
	_grid[pos._x][pos._y]._traversable = isTraversable;
}

bool IDAStar::findPath(Metrics& metrics)
{
	float g = 0.0f;
	_nrOfPathNodes = 0;																//It's 1 because there's an offset in the loop later.
	Vec2D currentPos = _start;
	float threshold = getHeuristicDistance(_start, _goal);

	while (currentPos != _goal)
	{
		currentPos = evaluateNode(_start, 0.0f, threshold);
		threshold = _grid[currentPos._x][currentPos._y]._gCost + getHeuristicDistance(currentPos, _goal);
		if (threshold >= _width * _height)
		{
			return false;
		}
	}
	while (currentPos != _start)													//traces the route back to start
	{
		_nrOfPathNodes++;
		currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	_path = new Vec2D[_nrOfPathNodes];
	int c = 0;
	currentPos = _goal;
	while (currentPos != _start)													//traces the route back to start
	{
		_path[c++] = currentPos;
		currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}

	return true;
}