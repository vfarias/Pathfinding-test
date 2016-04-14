#pragma once
#include "Pathfinding.h"
/*
Basic A* algorithm operating on a map with varying tile cost.
Should be relatively easy to expand for more specialized behavior
--Victor
*/
class AStar : public Pathfinding
{
public:
private:
	struct Node
	{
		Vec2D _position;
		__int8 _open;										//0 = not checked, 1 = open, 2 = closed
		__int16 _tileCost;									//cost of traversing the individual tile
		float _gCost, _hCost;								//distance from start and heuristic to goal, respectively
		Node* _parent;										//the path back to the start node 
		Node()
		{
			_position = {0, 0};
			_hCost = -1.0f;
			_gCost = -1.0f;
			_open = 0;
			_parent = nullptr;
			_tileCost = 0;
		}
		Node(__int16 x, __int16 y, float h = -1.0f)
		{
			_position = {x, y};
			_hCost = h;
			_gCost = -1.0f;
			_open = 0;
			_parent = nullptr;
			_tileCost = 0;
		}
		~Node()
		{}
		bool operator<(const Node& comp)
		{
			return (_gCost + _hCost) < (comp._gCost + comp._hCost);
		}
		bool operator>(const Node& comp)
		{
			return (_gCost + _hCost) > (comp._gCost + comp._hCost);
		}
	};
	Node** _grid;
	//std::vector<Node> _openQueue;							//A priority queue for open nodes
	Heap<Node> _openQueue;
	void calculateHCost(Vec2D pos);
	void calculateGCost(Vec2D parentPos, Vec2D currentPos);
public:
	AStar();
	AStar(int width, int height, Vec2D start, Vec2D goal, Heuristic heuristic = MANHATTAN);
	AStar(int width, int height, Heuristic heuristic = MANHATTAN);
	virtual ~AStar();
	void setTileCost(Vec2D pos, int cost = 1);
	int getTileCost(Vec2D pos)const;
	void cleanMap();
	bool findPath(Metrics& metrics);
};