

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
	if (pos == _goal || (g + getHeuristicDistance(pos, _goal)) - threshold > 0.0f)
	{
		return pos;
	}
	Vec2D minPos = {-1, -1};
	float minValue = -1.0f;
	for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
	{
		Vec2D checkedPos = pos + NEIGHBOUR_OFFSETS[i];
		Vec2D foundPos = {-1,-1};
		if ((_grid[pos._x][pos._y]._parent == nullptr || checkedPos != _grid[pos._x][pos._y]._parent->_position)
			&& isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._traversable)
		{

			float tileDist = 1;
			if (i >= 4)
			{
				tileDist = SQRT2;
			}
			if (_grid[checkedPos._x][checkedPos._y]._gCost <= 0 || (g + tileDist) - _grid[checkedPos._x][checkedPos._y]._gCost < 0.0000001f)
			{
				_grid[checkedPos._x][checkedPos._y]._parent = &_grid[pos._x][pos._y];
				foundPos = evaluateNode(checkedPos, g + tileDist, threshold);
			}
			
			if (isPositionValid(foundPos))
			{
				if (foundPos == _goal)
				{
					return foundPos;
				}
				float foundValue = _grid[foundPos._x][foundPos._y]._gCost + getHeuristicDistance(foundPos, _goal);
				if (/*foundValue > threshold && */(foundValue < minValue || minValue < 0) ||
					(minValue - foundValue < 0.0f && _grid[foundPos._x][foundPos._y]._gCost > _grid[minPos._x][minPos._y]._gCost))
				{
					minValue = foundValue;
					minPos = foundPos;
				}
			}
		}
	}
	return minPos;
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
		if (threshold >= _width * _height || !isPositionValid(currentPos))
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