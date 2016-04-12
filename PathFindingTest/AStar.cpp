#include "AStar.h"

bool AStar::isPositionValid(Vec2D pos)
{
	return pos._x >= 0 && pos._x < _width && pos._y >= 0 && pos._y < _height;
}

//Calculates h based on the distance to the goal
void AStar::calculateHCost(Vec2D pos)
{

	_grid[pos._x][pos._y]._hCost = getHeuristicDistance(pos, _goal) * _hWeight;
}

//calculates g by adding the preceding nodes g-cost to the current tilecost.
void AStar::calculateGCost(Vec2D parentPos, Vec2D currentPos)
{
	float g = 0.0f;
	//Euclidean heuristic is locked to octile movement for now.
	switch (_heuristicType)
	{
	case AStar::MANHATTAN:
	case AStar::CHEBYSHEV:
		g = _grid[parentPos._x][parentPos._y]._gCost + _grid[currentPos._x][currentPos._y]._tileCost;
		break;
	case AStar::OCTILE:
	case AStar::EUCLIDEAN:
		if (parentPos._x == currentPos._x || parentPos._y == currentPos._y)
		{
			g = _grid[parentPos._x][parentPos._y]._gCost + _grid[currentPos._x][currentPos._y]._tileCost;
		}
		else
		{
			g = _grid[parentPos._x][parentPos._y]._gCost + SQRT2 * _grid[currentPos._x][currentPos._y]._tileCost;
		}
		break;
	default:
		break;
	}

	// tileCost <= 0 means unwalkable. open == 0 means no previous gCost, meaning g is automatically better.
	if (_grid[currentPos._x][currentPos._y]._tileCost > 0 && (_grid[currentPos._x][currentPos._y]._open == 0 || _grid[currentPos._x][currentPos._y]._gCost > g))
	{
		_grid[currentPos._x][currentPos._y]._open = 1;
		_grid[currentPos._x][currentPos._y]._gCost = g;
		_grid[currentPos._x][currentPos._y]._parent = &_grid[parentPos._x][parentPos._y];
		_openQueue.insert(_grid[currentPos._x][currentPos._y]);										//insert should logically fit outside the function, but it works better with the if-check here.
	}
}

AStar::AStar()
{
	_pathLength = 0;
	_path = nullptr;
	_width = 0;
	_height = 0;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = MANHATTAN;
	_hWeight = 0;
	_openQueue = Heap<Node>();
	_grid = nullptr;
}

/*
Sets grid size, start- and goal positions and heuristic used for the pathfinding algorithm
*/
AStar::AStar(int width, int height, Vec2D start, Vec2D goal, Heuristic heuristic, int hWeight)
{
	_pathLength = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = start;
	_goal = goal;
	_heuristicType = heuristic;
	_hWeight = hWeight;
	_openQueue = Heap<Node>();
	_grid = new Node*[_width];
	for (__int16 i = 0; i < _width; i++)
	{
		_grid[i] = new Node[_height];
		for (__int16 j = 0; j < _height; j++)
		{
			_grid[i][j] = Node(i, j);
			_grid[i][j]._open = 0;
			//calculateHCost({i,j});
		}
	}
}


/*
Sets grid size and heuristic used for the pathfinding algorithm
*/
AStar::AStar(int width, int height, Heuristic heuristic, int hWeight)
{
	_pathLength = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = heuristic;
	_hWeight = hWeight;
	_openQueue = Heap<Node>();
	_grid = new Node*[_width];
	for (__int16 i = 0; i < _width; i++)
	{
		_grid[i] = new Node[_height];
		for (__int16 j = 0; j < _height; j++)
		{
			_grid[i][j] = Node(i, j);
			_grid[i][j]._open = 0;
			//calculateHCost({i,j});
		}
	}
}

AStar::~AStar()
{
	for (__int16 i = 0; i < _width; i++)
	{
		delete[] _grid[i];
	}
	delete[] _grid;
	delete[] _path;
}

void AStar::setTileCost(Vec2D pos, int cost)
{
	_grid[pos._x][pos._y]._tileCost = cost;
}

void AStar::setStartPosition(Vec2D pos)
{
	_start = pos;
}

void AStar::setGoalPosition(Vec2D pos)
{
	_goal = pos;
}

int AStar::getTileCost(Vec2D pos) const
{
	return _grid[pos._x][pos._y]._tileCost;
}

Vec2D * AStar::getPath() const
{
	return _path;
}

int AStar::getPathLength() const
{
	return _pathLength;
}

float AStar::getHeuristicDistance(Vec2D start, Vec2D goal) const
{
	float h = 0;
	short x = abs(goal._x - start._x);						//horizontal distance to goal
	short y = abs(goal._y - start._y);						//vertical distance to goal
	switch (_heuristicType)
	{
	case AStar::MANHATTAN:
		h = (float)(x + y);
		break;
	case AStar::CHEBYSHEV:
		h = (float)(std::min(x, y) + abs(x - y));
		break;
	case AStar::OCTILE:
		h = SQRT2 * std::min(x, y) + abs(x - y);
		break;
	case AStar::EUCLIDEAN:
		h = (float)(sqrt(x * x + y * y));
		break;
	default:
		break;
	}
	return h;
}

void AStar::cleanMap()
{
	delete[] _path;
	_path = nullptr;
	_pathLength = 0;
	for (__int16 i = 0; i < _width; i++)
	{
		for (__int16 j = 0; j < _height; j++)
		{
			_grid[i][j]._open = 0;
			_grid[i][j]._gCost = 0;
			_grid[i][j]._hCost = 0;
			_grid[i][j]._parent = nullptr;
		}
	}
	_openQueue.empty();
	_openQueue = Heap<Node>();
}

/*
Make Everything ready for the algorithm to run
*/
void AStar::init(Vec2D start, Vec2D goal)
{
	cleanMap();
	setStartPosition(start);
	setGoalPosition(goal);
}

/*
Runs the actual A* algorithm.
*/
bool AStar::findPath()
{
	if (_goal == _start)
	{
		return false;
	}
	_pathLength = 0;																//It's 1 because there's an offset in the loop later.
	Vec2D currentPos = _start;
	_grid[_start._x][_start._y]._open = 2;

	while (currentPos != _goal)														//loops until a path has been found
	{
		for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
		{
			Vec2D checkedPos = currentPos + NEIGHBOUR_OFFSETS[i];
			if (isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._open != 2 && _grid[checkedPos._x][checkedPos._y]._tileCost > 0 &&	 //checks for borders and already visited
				_grid[checkedPos._x][currentPos._y]._tileCost > 0 && _grid[currentPos._x][checkedPos._y]._tileCost > 0)								//checks for corners
			{
				calculateHCost(checkedPos);											//As the program works now, h must be calculated before g.
				calculateGCost(currentPos, checkedPos);
			}
		}
		if (_openQueue.size() <= 0)
		{
			return false;
		}
		else
		{
			currentPos = _openQueue.removeMin()._position;
			while (_grid[currentPos._x][currentPos._y]._open == 2)
			{
				if (_openQueue.size() <= 0)
				{
					return false;
				}
				currentPos = _openQueue.removeMin()._position;
			}
			_grid[currentPos._x][currentPos._y]._open = 2;
		}
	}
	while (currentPos != _start)													//traces the route back to start
	{
		_pathLength++;
		currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	_path = new Vec2D[_pathLength];
	int c = 0;
	currentPos = _goal;
	while (currentPos != _start)													//traces the route back to start
	{
		_path[c++] = currentPos;
		currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	//	_path[c++] = currentPos;														//Excluding start position since it should already be known
	return true;
}

bool AStar::findPath(Metrics& metrics)
{
	if (_goal == _start)
	{
		return false;
	}
	_pathLength = 0;																//It's 1 because there's an offset in the loop later.
	Vec2D currentPos = _start;
	_grid[_start._x][_start._y]._open = 2;

	while (currentPos != _goal)														//loops until a path has been found
	{
		metrics.addExpandedNode(currentPos);
		for (int i = 0; i < 8 && (_heuristicType != MANHATTAN || i < 4); i++)		//Manhattan skips diagonals 
		{
			Vec2D checkedPos = currentPos + NEIGHBOUR_OFFSETS[i];
			if (isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._open != 2 && _grid[checkedPos._x][checkedPos._y]._tileCost > 0 &&	 //checks for borders and already visited
				_grid[checkedPos._x][currentPos._y]._tileCost > 0 && _grid[currentPos._x][checkedPos._y]._tileCost > 0)								//checks for corners
			{
				bool openedBefore = true;
				if (_grid[checkedPos._x][checkedPos._y]._open == 0)			//check that node is not already in open list
				{
					calculateHCost(checkedPos);						//As the program works now, h must be calculated before g.
					openedBefore = false;
				}												
				calculateGCost(currentPos, checkedPos);
				if (!openedBefore && _grid[checkedPos._x][checkedPos._y]._open == 1)	//Check that node was added to open list
				{
					metrics.addOpenedNode(checkedPos);
				}
			}
		}
		if (_openQueue.size() <= 0)
		{
			return false;
		}
		else
		{
			currentPos = _openQueue.removeMin()._position;
			while (_grid[currentPos._x][currentPos._y]._open == 2)
			{
				if (_openQueue.size() <= 0)
				{
					return false;
				}
				currentPos = _openQueue.removeMin()._position;
			}
			_grid[currentPos._x][currentPos._y]._open = 2;
		}
	}
	while (currentPos != _start)													//traces the route back to start
	{
		_pathLength++;
		currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	_path = new Vec2D[_pathLength];
	int c = 0;
	currentPos = _goal;
	while (currentPos != _start)													//traces the route back to start
	{
		_path[c++] = currentPos;
		currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	//	_path[c++] = currentPos;														//Excluding start position since it should already be known
	return true;
}


void AStar::printMap()
{
	std::fstream file;
	file.open("aimap.txt");
	for (__int16 i = 0; i < _width; i++)
	{
		for (__int16 j = 0; j < _height; j++)
		{
			//if (_grid[i][j]._tileCost <= 0)
			//{
			//	file << 0 << "\t";
			//}
			//else
			//{
			file << (int)_grid[i][j]._tileCost << "\t";
			//	}
		}
		file << "\n";
	}
	file.close();
}