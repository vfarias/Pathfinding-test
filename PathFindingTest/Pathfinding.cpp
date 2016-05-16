#include "Pathfinding.h"

bool Pathfinding::isPositionValid(const Vec2D& pos)
{
	return pos._x >= _position._x && pos._x < _position._x + _width && pos._y >= _position._y && pos._y < _position._y + _height;
}

Pathfinding::Pathfinding()
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = 0;
	_height = 0;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = MANHATTAN;
	_position = {0,0};
	_grid = nullptr;
}

/*
Sets grid size, start- and goal positions and heuristic used for the pathfinding algorithm
*/
Pathfinding::Pathfinding(int width, int height, AStarNode** grid, Vec2D start, Vec2D goal, Vec2D position, Heuristic heuristic)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = start;
	_goal = goal;
	_heuristicType = heuristic;
	_position = position;
	_grid = grid;
	//for (__int16 i = _position._x; i < _position._x + _width; i++)
	//{
	//	for (__int16 j = _position._y; j < _position._y + _height; j++)
	//	{
	//		_grid[i][j]._open = 0;
	//		_grid[i][j]._gCost = 0.0f;
	//		_grid[i][j]._hCost = 0.0f;
	//		_grid[i][j]._parent = nullptr;
	//	}
	//}
}


/*
Sets grid size and heuristic used for the pathfinding algorithm
*/
Pathfinding::Pathfinding(int width, int height, AStarNode** grid, Vec2D position, Heuristic heuristic)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = heuristic;
	_position = position;
	_grid = grid;
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
}

Pathfinding::~Pathfinding()
{
	delete[] _path;
	_path = nullptr;
}

void Pathfinding::setPosition(const Vec2D pos)
{
	_position = pos;
}

void Pathfinding::setStartPosition(const Vec2D start)
{
	_start = start;
}

void Pathfinding::setGoalPosition(const Vec2D goal)
{
	_goal = goal;
}

void Pathfinding::setTraversable(Vec2D pos, bool isTraversable)
{
	_grid[pos._x][pos._y]._traversable = isTraversable;
}

bool Pathfinding::isTraversable(Vec2D pos) const
{
	return _grid[pos._x][pos._y]._traversable;
}

Vec2D * Pathfinding::getPath() const
{
	return _path;
}

int Pathfinding::getNrOfPathNodes() const
{
	return _nrOfPathNodes;
}

double Pathfinding::getHeuristicDistance(Vec2D start, Vec2D goal) const
{
	double h = 0;
	int x = abs(goal._x - start._x);						//horizontal distance to goal
	int y = abs(goal._y - start._y);						//vertical distance to goal
	switch (_heuristicType)
	{
	case Pathfinding::MANHATTAN:
		h = (double)(x + y);
		break;
	case Pathfinding::CHEBYSHEV:
		h = (double)(std::min(x, y) + abs(x - y));
		break;
	case Pathfinding::OCTILE:
		h = (M_SQRT2 * std::min(x, y) + abs(x - y));
		break;
	case Pathfinding::EUCLIDEAN:
		h = sqrt(x * x + y * y);
		break;
	default:
		break;
	}
	return h;
}

void Pathfinding::cleanMap()
{
	delete[] _path;
	_path = nullptr;
	_nrOfPathNodes = 0;
}

/*
Make Everything ready for the algorithm to run
*/
void Pathfinding::init(Vec2D start, Vec2D goal)
{
	cleanMap();
	setStartPosition(start);
	setGoalPosition(goal);
}