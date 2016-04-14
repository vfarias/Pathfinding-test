#pragma once
#include "AStar.h"

class HPAStar : public Pathfinding
{
private:
	struct Node
	{
		Vec2D _position;
		float _gCost, _hCost;					//distance from start and heuristic to goal, respectively
		Node* _parent;							//the path back to the start node 
		Node* _edge;							//The correspending node across the cluster border
		Node()
		{
			_position = {0, 0};
			_parent = nullptr;
			_edge = nullptr;
		}
		Node(__int16 x, __int16 y, float h = -1.0f)
		{
			_position = {x, y};
			_parent = nullptr;
			_edge = nullptr;
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
	struct Cluster
	{
		Vec2D _position;
		Node** _internalNodes;						//pointers to all nodes within the cluster. All nodes are placed along the edges
		int** _internalPathLengths;
		int _nrOfInternalNodes;
	};

	AStar* _aStar;					//Used for the abstracted pathfinding
	int _clusterSize;
};