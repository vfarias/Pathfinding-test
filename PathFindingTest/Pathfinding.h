#pragma once
#include <cmath>
#include <algorithm>
#include <fstream>
#include "Heap.h"
#include "AIUtil.h"
#include "Metrics.h"
/*
Base class to the pathfinding algorithms
*/
class Pathfinding
{
public:
	/*
	Different heuristic used for estimating the distance to the goal
	MANHATTAN: No diagonal movement
	CHEBYSHEV: Diagonal movement has a cost of 1
	OCTILE: Diagonal movement costs sqrt(2)
	EUCLIDEAN: Calculates the distance in a straight line to the goal.
	*/
	enum Heuristic
	{
		MANHATTAN, CHEBYSHEV, OCTILE, EUCLIDEAN
	};
protected:
	int _nrOfPathNodes;
	Vec2D* _path;											//An ordered array moving from goal to start
	__int16 _width, _height;								//Size of the grid
	Vec2D _start, _goal;
	Heuristic _heuristicType;

	bool isPositionValid(Vec2D pos);						//Checks if position is within the grid
	virtual void calculateHCost(Vec2D pos) = 0;							//Sets the approximate distance to the goal node according to the chosen heuristic
	virtual void calculateGCost(Vec2D parentPos, Vec2D currentPos) = 0; //Sets cost from the start node
public:
	Pathfinding();
	Pathfinding(int width, int height, Vec2D start, Vec2D goal, Heuristic heuristic = MANHATTAN);
	Pathfinding(int width, int height, Heuristic heuristic = MANHATTAN);
	virtual ~Pathfinding();
	void setStartPosition(Vec2D start);
	void setGoalPosition(Vec2D goal);
	Vec2D* getPath() const;
	int getNrOfPathNodes() const;
	float getHeuristicDistance(Vec2D start, Vec2D goal) const;
	void cleanMap();
	void init(Vec2D start, Vec2D goal);
	virtual bool findPath(Metrics& metrics) = 0;
};