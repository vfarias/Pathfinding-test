#include "ThetaStar.h"

bool ThetaStar::isPositionValid(Vec2D pos)
{
	return pos._x >= 0 && pos._x < _width && pos._y >= 0 && pos._y < _height;
}

//Calculates h based on the distance to the goal
void ThetaStar::calculateHCost(Vec2D pos)
{

	_grid[pos._x][pos._y]._hCost = getHeuristicDistance(pos, _goal) * _hWeight;
}

//calculates g by adding the preceding nodes g-cost to the current tilecost.
void ThetaStar::calculateGCost(Vec2D parentPos, Vec2D currentPos)
{
	float g = _grid[parentPos._x][parentPos._y]._gCost + sqrt(pow(parentPos._x - currentPos._x, 2) + pow(parentPos._y - currentPos._y, 2));

	// tileCost <= 0 means unwalkable. open == 0 means no previous gCost, meaning g is automatically better.
	if (_grid[currentPos._x][currentPos._y]._tileCost > 0 && (_grid[currentPos._x][currentPos._y]._open == 0 || _grid[currentPos._x][currentPos._y]._gCost > g))
	{
		_grid[currentPos._x][currentPos._y]._open = 1;
		_grid[currentPos._x][currentPos._y]._gCost = g;
		_grid[currentPos._x][currentPos._y]._parent = &_grid[parentPos._x][parentPos._y];
		_openQueue.insert(_grid[currentPos._x][currentPos._y]);										//insert should logically fit outside the function, but it works better with the if-check here.
	}
}

bool ThetaStar::lineOfSightBresenham(Vec2D parentPos, Vec2D currentPos)
{
	int x0 = currentPos._x;
	int y0 = currentPos._y;
	int x1 = parentPos._x;
	int y1 = parentPos._y;
	int dY = y1 - y0;
	int dX = x1 - x0;
	int f = 0;
	int sY = 1;
	int sX = 1;
	if (dY < 0)
	{
		dY = -dY;
		sY = -1;
	}
	if (dX < 0)
	{
		dX = -dX;
		sX = -1;
	}
	if (dX >= dY)
	{
		while (x0 != x1)
		{
			f += dY;
			if (f >= dX)
			{
				if (_grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._tileCost <= 0)			//Found a wall
				{
					return false;
				}
				y0 += sY;
				f -= dX;
			}
			if (f != 0 && _grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._tileCost <= 0)
			{
				return false;
			}
			if (dY == 0 && _grid[x0 + ((sX - 1) / 2)][y0]._tileCost <= 0 && _grid[x0 + ((sX - 1) / 2)][y0 - 1]._tileCost <= 0)
			{
				return false;
			}
			x0 += sX;
		}
	} else
	{
		while (y0 != y1)
		{
			f += dX;
			if (f >= dY)
			{
				if (_grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._tileCost <= 0)			//Found a wall
				{
					return false;
				}
				x0 += sX;
				f -= dY;
			}
			if (f != 0 && _grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._tileCost <= 0)
			{
				return false;
			}
			if (dX == 0 && _grid[x0][y0]._tileCost <= 0 && _grid[x0 - 1][y0 - 1]._tileCost <= 0)
			{
				return false;
			}
			y0 += sY;
		}
	}
	return true;
}

/*
	A tile is crossing the vector defined by parentPos and currentPos 
	if any corners are on different sides of the vector the tile is crossing the line
*/
bool ThetaStar::lineOfSightRay(Vec2D parentPos, Vec2D currentPos)
{
	int x0 = currentPos._x;
	int y0 = currentPos._y;
	int x1 = parentPos._x;
	int y1 = parentPos._y;
	int dY = y1 - y0;
	int dX = x1 - x0;
	if (x0 > x1)
	{
		x0 = parentPos._x;
		x1 = currentPos._x;
	}
	if (y0 > y1)
	{
		y0 = parentPos._y;
		y1 = currentPos._y;
	}
	for (int i = x0; i <= x1; i++)
	{
		for (int j = y0; j <= y1; j++)
		{	
			float cross1 = (i - parentPos._x + 0.5) * dY - dX * (j - parentPos._y + 0.5);
			float cross2 = (i - parentPos._x - 0.5) * dY - dX * (j - parentPos._y - 0.5);
			float cross3 = (i - parentPos._x + 0.5) * dY - dX * (j - parentPos._y - 0.5);
			float cross4 = (i - parentPos._x - 0.5) * dY - dX * (j - parentPos._y + 0.5);
			int negCross = 0;
			if (cross1 < 0)
			{
				negCross++;
			}
			if (cross2 < 0)
			{
				negCross++;
			}
			if (cross3 < 0)
			{
				negCross++;
			}
			if (cross4 < 0)
			{
				negCross++;
			}
			if (_grid[i][j]._tileCost < 0 && negCross > 0 && negCross < 4)
			{
				return false;
			}
		}
	}
	return true;
}

ThetaStar::ThetaStar()
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
ThetaStar::ThetaStar(int width, int height, Vec2D start, Vec2D goal, Heuristic heuristic, int hWeight)
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
ThetaStar::ThetaStar(int width, int height, Heuristic heuristic, int hWeight)
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

ThetaStar::~ThetaStar()
{
	for (__int16 i = 0; i < _width; i++)
	{
		delete[] _grid[i];
	}
	delete[] _grid;
	delete[] _path;
}

void ThetaStar::setTileCost(Vec2D pos, int cost)
{
	_grid[pos._x][pos._y]._tileCost = cost;
}

void ThetaStar::setStartPosition(Vec2D pos)
{
	_start = pos;
}

void ThetaStar::setGoalPosition(Vec2D pos)
{
	_goal = pos;
}

int ThetaStar::getTileCost(Vec2D pos) const
{
	return _grid[pos._x][pos._y]._tileCost;
}

Vec2D * ThetaStar::getPath() const
{
	return _path;
}

int ThetaStar::getPathLength() const
{
	return _pathLength;
}

float ThetaStar::getHeuristicDistance(Vec2D start, Vec2D goal) const
{
	short x = abs(goal._x - start._x);						//horizontal distance to goal
	short y = abs(goal._y - start._y);						//vertical distance to goal
	return (float)(sqrt(x * x + y * y));
}

void ThetaStar::cleanMap()
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
void ThetaStar::init(Vec2D start, Vec2D goal)
{
	cleanMap();
	setStartPosition(start);
	setGoalPosition(goal);
}

/*
Runs the actual A* algorithm.
*/
bool ThetaStar::findPath()
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
			if (isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._open != 2 && _grid[checkedPos._x][checkedPos._y]._tileCost > 0)	 //checks for borders and already visited
			{
				calculateHCost(checkedPos);											//As the program works now, h must be calculated before g.
				
				Vec2D parentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
				if (lineOfSightBresenham(parentPos, currentPos))
				{
					calculateGCost(parentPos, checkedPos);
				}
				else
				{
					calculateGCost(currentPos, checkedPos);
				}
			}
		}
		if (_openQueue.size() <= 0)
		{
			return false;
		} else
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

bool ThetaStar::findPath(Metrics& metrics)
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
				if (_grid[currentPos._x][currentPos._y]._parent != nullptr)
				{
					Vec2D parentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
					if (lineOfSightRay(parentPos, checkedPos))
					{
						calculateGCost(parentPos, checkedPos);
					} else
					{
						calculateGCost(currentPos, checkedPos);
					}
				}
				else
				{
					calculateGCost(currentPos, checkedPos);
				}

				if (!openedBefore && _grid[checkedPos._x][checkedPos._y]._open == 1)	//Check that node was added to open list
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


void ThetaStar::printMap()
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