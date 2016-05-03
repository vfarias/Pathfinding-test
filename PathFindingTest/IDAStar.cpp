

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
{}
IDAStar::IDAStar(int width, int height, AStarNode** grid, Heuristic heuristicType) :
	Pathfinding(width, height, grid, {0,0}, heuristicType)
{}
IDAStar::IDAStar(int width, int height, Vec2D start, Vec2D goal, AStarNode** grid, Heuristic heuristicType) :
	Pathfinding(width, height, grid, start, goal, {0,0}, heuristicType)
{}

IDAStar::~IDAStar()
{
}

Vec2D IDAStar::evaluateNode(Vec2D pos, float g, float threshold, Metrics& metrics)
{
	metrics.countExpansion();
	_grid[pos._x][pos._y]._gCost = g;
	float f = g + getHeuristicDistance(pos, _goal);
	if (pos == _goal || f - threshold > 0.1f )
	{
		return pos;
	}
	Vec2D minPos = {-1, -1};
	float minValue = -1.0f;
	Vec2D parentNeighbour1 = {0, 0};
	Vec2D parentNeighbour2 = {0, 0};
	Vec2D parentPos = {0, 0};
	if (_grid[pos._x][pos._y]._parent != nullptr)
	{
		parentPos = _grid[pos._x][pos._y]._parent->_position;
		if (parentPos._x != 0 && parentPos._y != 0)
		{
			parentNeighbour1 = {parentPos._x, 0};
			parentNeighbour2 = {0, parentPos._y};
		}
		else if (parentPos._x != 0)
		{
			parentNeighbour1 = {parentPos._x, 1};
			parentNeighbour2 = {parentPos._x, -1};
		}
		else
		{
			parentNeighbour1 = {-1, parentPos._y};
			parentNeighbour2 = {1, parentPos._y};
		}
	}
	for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
	{
		Vec2D checkedPos = pos + NEIGHBOUR_OFFSETS[i];
		if ((_grid[pos._x][pos._y]._parent == nullptr || (checkedPos != parentPos && checkedPos != parentNeighbour1 && checkedPos != parentNeighbour2))
			&& isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._traversable)
		{
			Vec2D foundPos = {-1, -1};
			float tileDist = 1;
			if (i >= 4)
			{
				tileDist = SQRT2;
			}
			if (_grid[checkedPos._x][checkedPos._y]._gCost <= 0 || _grid[checkedPos._x][checkedPos._y]._gCost - (g + tileDist) > -0.1f )
			{
				_grid[checkedPos._x][checkedPos._y]._parent = &_grid[pos._x][pos._y];
				foundPos = evaluateNode(checkedPos, g + tileDist, threshold, metrics);
			}
			
			if (isPositionValid(foundPos))
			{
				if (foundPos == _goal)
				{
					return foundPos;
				}
				float foundValue = _grid[foundPos._x][foundPos._y]._gCost + getHeuristicDistance(foundPos, _goal);
				if ((minValue - foundValue > 0.1f || minValue < 0.0f) && foundValue - threshold > 0.1) /*|| (abs(minValue - foundValue) < 0.0001f && _grid[minPos._x][minPos._y]._gCost > _grid[foundPos._x][foundPos._y]._gCost)*/
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
		currentPos = evaluateNode(_start, 0.0f, threshold, metrics);
		if (!isPositionValid(currentPos))
		{
			return false;
		}
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
	metrics.setPathNodes(_path, _nrOfPathNodes, _grid[_goal._x][_goal._y]._gCost);
	return true;
}