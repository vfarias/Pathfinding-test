#include "Pathfinding.h"

bool Pathfinding::isPositionValid(Vec2D pos)
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
}

/*
Sets grid size, start- and goal positions and heuristic used for the pathfinding algorithm
*/
Pathfinding::Pathfinding(int width, int height, Vec2D start, Vec2D goal, Vec2D position, Heuristic heuristic)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = start;
	_goal = goal;
	_heuristicType = heuristic;
	_position = position;
}


/*
Sets grid size and heuristic used for the pathfinding algorithm
*/
Pathfinding::Pathfinding(int width, int height, Vec2D position, Heuristic heuristic)
{
	_nrOfPathNodes = 0;
	_path = nullptr;
	_width = width;
	_height = height;
	_start = {0,0};
	_goal = {0,0};
	_heuristicType = heuristic;
	_position = position;
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

Vec2D * Pathfinding::getPath() const
{
	return _path;
}

int Pathfinding::getNrOfPathNodes() const
{
	return _nrOfPathNodes;
}

float Pathfinding::getHeuristicDistance(Vec2D start, Vec2D goal) const
{
	float h = 0;
	short x = abs(goal._x - start._x);						//horizontal distance to goal
	short y = abs(goal._y - start._y);						//vertical distance to goal
	switch (_heuristicType)
	{
	case Pathfinding::MANHATTAN:
		h = (float)(x + y);
		break;
	case Pathfinding::CHEBYSHEV:
		h = (float)(std::min(x, y) + abs(x - y));
		break;
	case Pathfinding::OCTILE:
		h = SQRT2 * std::min(x, y) + abs(x - y);
		break;
	case Pathfinding::EUCLIDEAN:
		h = (float)(sqrt(x * x + y * y));
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