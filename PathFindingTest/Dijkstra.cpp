#include "Dijkstra.h"

void Dijkstra::calculateGCost(Vec2D parentPos, Vec2D currentPos)
{
	float g = 0.0f;
	//Euclidean heuristic is locked to octile movement for now.
	switch (_heuristicType)
	{
	case Pathfinding::MANHATTAN:
	case Pathfinding::CHEBYSHEV:
		g = _grid[parentPos._x][parentPos._y]._gCost + 1;
		break;
	case Pathfinding::OCTILE:
	case Pathfinding::EUCLIDEAN:
		if (parentPos._x == currentPos._x || parentPos._y == currentPos._y)
		{
			g = _grid[parentPos._x][parentPos._y]._gCost + 1;
		} else
		{
			g = _grid[parentPos._x][parentPos._y]._gCost + SQRT2;
		}
		break;
	default:
		break;
	}
	if (_grid[currentPos._x][currentPos._y]._traversable && (_grid[currentPos._x][currentPos._y]._open == 0 || _grid[currentPos._x][currentPos._y]._gCost > g))
	{
		_grid[currentPos._x][currentPos._y]._open = 1;
		_grid[currentPos._x][currentPos._y]._gCost = g;
		_grid[currentPos._x][currentPos._y]._parent = &_grid[parentPos._x][parentPos._y];
		_openQueue.insert(&_grid[currentPos._x][currentPos._y]);
	}
}

Dijkstra::Dijkstra()
	:Pathfinding()
{
	_openQueue = Heap<AStarNode*>();
}

Dijkstra::Dijkstra(int width, int height, Vec2D start, Vec2D goal, AStarNode ** grid, Heuristic heuristic)
	:Pathfinding(width, height, grid, start, goal)
{
	_openQueue = Heap<AStarNode*>();
}

Dijkstra::Dijkstra(int width, int height, AStarNode ** grid, Heuristic heuristic)
	: Pathfinding(width, height, grid, {0,0}, heuristic)
{
	_openQueue = Heap<AStarNode*>();
}

Dijkstra::~Dijkstra()
{}

void Dijkstra::cleanMap()
{
	Pathfinding::cleanMap();
	for (__int16 i = _position._x; i < _position._x + _width; i++)
	{
		for (__int16 j = _position._y; j < _position._y + _height; j++)
		{
			_grid[i][j]._open = 0;
			_grid[i][j]._gCost = 0.0f;
			_grid[i][j]._hCost = 0.0f;
			_grid[i][j]._parent = nullptr;
		}
	}
	_openQueue.empty();
	_openQueue = Heap<AStarNode*>();
}

bool Dijkstra::findPath(Metrics & metrics)
{
	if (_goal == _start)
	{
		return false;
	}
	_nrOfPathNodes = 0;																//It's 1 because there's an offset in the loop later.
	Vec2D currentPos = _start;
	_grid[_start._x][_start._y]._open = 2;

	while (currentPos != _goal)														//loops until a path has been found
	{
		metrics.addExpandedNode(currentPos);
		for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
		{
			Vec2D checkedPos = currentPos + NEIGHBOUR_OFFSETS[i];
			if (isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._open != 2 && _grid[checkedPos._x][checkedPos._y]._traversable /*&&	 //checks for borders and already visited
				_grid[checkedPos._x][currentPos._y]._traversable && _grid[currentPos._x][checkedPos._y]._traversable*/)							//checks for corners
			{
				bool openedBefore = true;
				if (_grid[checkedPos._x][checkedPos._y]._open == 0)
				{
					openedBefore = false;
				}
				calculateGCost(currentPos, checkedPos);
				if (_grid[checkedPos._x][checkedPos._y]._open == 1 && !openedBefore)	//Check that node was added to open list
				{
					metrics.addOpenedNode(checkedPos);
				}
			}
		}
		if (_openQueue.size() <= 0)
		{
			return false;
		} else
		{
			currentPos = _openQueue.removeMin()->_position;
			while (_grid[currentPos._x][currentPos._y]._open == 2)
			{
				if (_openQueue.size() <= 0)
				{
					return false;
				}
				currentPos = _openQueue.removeMin()->_position;
			}
			_grid[currentPos._x][currentPos._y]._open = 2;
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
	//	_path[c++] = currentPos;														//Excluding start position since it should already be known
	metrics.setPathNodes(_path, _nrOfPathNodes, _grid[_goal._x][_goal._y]._gCost);
	return true;
}

float Dijkstra::getPathLength()
{
	return 0.0f;
}
