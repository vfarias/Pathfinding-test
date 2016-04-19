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
	AStarNode(__int16 x, __int16 y, float h = -1.0f)
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