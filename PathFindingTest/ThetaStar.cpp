#include "ThetaStar.h"

//calculates g by adding the preceding nodes g-cost to the current tilecost.
void ThetaStar::calculateGCost(Vec2D parentPos, Vec2D currentPos)
{
	float g = _grid[parentPos._x][parentPos._y]._gCost + (float)sqrt(pow(parentPos._x - currentPos._x, 2) + pow(parentPos._y - currentPos._y, 2));

	// tileCost <= 0 means unwalkable. open == 0 means no previous gCost, meaning g is automatically better.
	if (_grid[currentPos._x][currentPos._y]._traversable && (_grid[currentPos._x][currentPos._y]._open == 0 || _grid[currentPos._x][currentPos._y]._gCost > g))
	{
		_grid[currentPos._x][currentPos._y]._open = 1;
		_grid[currentPos._x][currentPos._y]._gCost = g;
		_grid[currentPos._x][currentPos._y]._parent = &_grid[parentPos._x][parentPos._y];
		_openQueue.insert(&_grid[currentPos._x][currentPos._y]);										//insert should logically fit outside the function, but it works better with the if-check here.
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
				if (!_grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._traversable)			//Found a wall
				{
					return false;
				}
				y0 += sY;
				f -= dX;
			}
			if (f != 0 && !_grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._traversable)
			{
				return false;
			}
			if (dY == 0 && !_grid[x0 + ((sX - 1) / 2)][y0]._traversable && !_grid[x0 + ((sX - 1) / 2)][y0 - 1]._traversable)
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
				if (!_grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._traversable)			//Found a wall
				{
					return false;
				}
				x0 += sX;
				f -= dY;
			}
			if (f != 0 && !_grid[x0 + ((sX - 1) / 2)][y0 + ((sY - 1) / 2)]._traversable)
			{
				return false;
			}
			if (dX == 0 && !_grid[x0][y0]._traversable && !_grid[x0 - 1][y0 - 1]._traversable)
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
			float cross1 = (i - parentPos._x + 0.5f) * dY - dX * (j - parentPos._y + 0.5f);
			float cross2 = (i - parentPos._x - 0.5f) * dY - dX * (j - parentPos._y - 0.5f);
			float cross3 = (i - parentPos._x + 0.5f) * dY - dX * (j - parentPos._y - 0.5f);
			float cross4 = (i - parentPos._x - 0.5f) * dY - dX * (j - parentPos._y + 0.5f);
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
			if (!_grid[i][j]._traversable && negCross > 0 && negCross < 4)
			{
				return false;
			}
		}
	}
	return true;
}

ThetaStar::ThetaStar()
	:AStar()
{
}

/*
Sets grid size, start- and goal positions and heuristic used for the pathfinding algorithm
*/
ThetaStar::ThetaStar(int width, int height, AStarNode** grid, Vec2D start, Vec2D goal, Heuristic heuristic)
	:AStar(width, height, {0,0}, start, goal, grid, heuristic)
{}


/*
Sets grid size and heuristic used for the pathfinding algorithm
*/
ThetaStar::ThetaStar(int width, int height, AStarNode** grid, Heuristic heuristic)
	:AStar(width, height, {0,0}, grid, heuristic)
{}

ThetaStar::~ThetaStar()
{}

bool ThetaStar::findPath(Metrics& metrics)
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
			if (isPositionValid(checkedPos) && _grid[checkedPos._x][checkedPos._y]._open != 2 && _grid[checkedPos._x][checkedPos._y]._traversable &&	 //checks for borders and already visited
				(_grid[checkedPos._x][currentPos._y]._traversable || _grid[currentPos._x][checkedPos._y]._traversable))								//checks for corners
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
	return true;
}