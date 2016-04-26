#pragma once
#include "AIUtil.h"

struct BaseNode
{
	Vec2D _position;
	bool _traversable;
};
struct AStarNode : public BaseNode
{
	__int8 _open;										//0 = not checked, 1 = open, 2 = closed
	float _gCost, _hCost;								//distance from start and heuristic to goal, respectively
	AStarNode* _parent;									//the path back to the start node 
	AStarNode()
	{
		_position = {0, 0};
		_hCost = -1.0f;
		_gCost = -1.0f;
		_open = 0;
		_parent = nullptr;
	}
	AStarNode(__int16 x, __int16 y, float h = -1.0f)  //Remove the weighting
	{
		_position = {x, y};
		_hCost = h;
		_gCost = -1.0f;
		_open = 0;
		_parent = nullptr;
	}
	~AStarNode()
	{}
	bool operator<(const AStarNode& comp)
	{
		return (_gCost + _hCost) < (comp._gCost + comp._hCost);
	}
	bool operator>(const AStarNode& comp)
	{
		return (_gCost + _hCost) > (comp._gCost + comp._hCost);
	}
};

//TODO: AStarNode might be general enough to serve as the base node.
//Reevaluate once the needs for HPA* is more locked down
struct HPANode : public BaseNode
{
	__int8 _open;
	float _gCost, _hCost;					//distance from start and heuristic to goal, respectively
	HPANode* _parent;						//the path back to the start node 
	HPANode* _edge;							//The correspending node across the cluster border
	HPANode* _edge2;						//Because fuck corners
	int _clusterIndex;						//which index the node has in Cluster::_internalNodes
	HPANode()
	{
		_position = {0, 0};
		_parent = nullptr;
		_edge = nullptr;
		_edge2 = nullptr;
		_open = 0;
		_clusterIndex = 0;
	}
	HPANode(__int16 x, __int16 y, float h = -1.0f)
	{
		_position = {x, y};
		_parent = nullptr;
		_edge = nullptr;
		_edge2 = nullptr;
		_open = 0;
		_clusterIndex = 0;
	}
	~HPANode()
	{}
	bool operator<(const HPANode& comp)
	{
		return (_gCost + _hCost) < (comp._gCost + comp._hCost);
	}
	bool operator>(const HPANode& comp)
	{
		return (_gCost + _hCost) > (comp._gCost + comp._hCost);
	}
};