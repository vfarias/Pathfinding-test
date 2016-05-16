

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

Vec2D IDAStar::evaluateNode(Vec2D pos, double g, double& f, double threshold, int iteration, Metrics& metrics)
{
	metrics.countExpansion();
	_grid[pos._x][pos._y]._gCost = g;
	double fTemp= g + getHeuristicDistance(pos, _goal);
	if (pos == _goal || fTemp - threshold > 0.00000001)
	{
		f = fTemp;
		return pos;
	}
	Vec2D minPos = {-1, -1};
	double minValue = -1.0f;
	Vec2D parentPos = {0, 0};
	if (_grid[pos._x][pos._y]._parent != nullptr)
	{
		parentPos = _grid[pos._x][pos._y]._parent->_position - _grid[pos._x][pos._y]._position;
	}
	for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
	{
		Vec2D checkedPos = pos + NEIGHBOUR_OFFSETS[i];
		if ((_grid[pos._x][pos._y]._parent == nullptr || (NEIGHBOUR_OFFSETS[i] != parentPos))
			&& isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._traversable)
		{
			Vec2D foundPos = {-1, -1};
			double tileDist = 1.0;
			if (i >= 4)
			{
				tileDist = M_SQRT2;
			}
			if (_grid[checkedPos._x][checkedPos._y]._open != iteration || (g + tileDist) < _grid[checkedPos._x][checkedPos._y]._gCost)
			{
				_grid[checkedPos._x][checkedPos._y]._parent = &_grid[pos._x][pos._y];
				_grid[checkedPos._x][checkedPos._y]._open = iteration;
				foundPos = evaluateNode(checkedPos, g + tileDist, f, threshold, iteration, metrics);
			}
			
			if (isPositionValid(foundPos))
			{
				if (foundPos == _goal)
				{
					return foundPos;
				}
				//float foundValue = _grid[foundPos._x][foundPos._y]._gCost + getHeuristicDistance(foundPos, _goal);
				if ((f < minValue || minValue < 0.0f) /*&& foundValue > threshold*/) /*|| (abs(minValue - foundValue) < 0.0001f && _grid[minPos._x][minPos._y]._gCost > _grid[foundPos._x][foundPos._y]._gCost)*/
				{
					minValue = f;
					minPos = foundPos;
				}
			}
		}
	}
	f = minValue;
	return minPos;
}

bool IDAStar::findPath(Metrics& metrics)
{
	double g = 0.0;
	_nrOfPathNodes = 0;																//It's 1 because there's an offset in the loop later.
	Vec2D currentPos = _start;

	double threshold = getHeuristicDistance(_start, _goal);
	int iteration = 1;
	while (currentPos != _goal)
	{
		double f = threshold;
		currentPos = evaluateNode(_start, 0.0f, f, threshold, iteration, metrics);
		iteration++;
		if (!isPositionValid(currentPos))
		{
			return false;
		}
 		threshold = f;
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