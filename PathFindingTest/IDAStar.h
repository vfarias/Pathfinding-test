

#pragma once

#include "AIUtil.h"

using namespace std;

class IDAStar
{
private:

	float _g;
	float _f;

	int _width;
	int _height;
	Vec2D _startPos;
	Vec2D _goalPos;

	bool isAtGoal(/*Node start*/);
	bool isPositionValid(Vec2D pos);
public:
	IDAStar();
	IDAStar(int width, int height, Vec2D startPos, Vec2D goalPos /*Heuristic, hWeight*/);
	~IDAStar();

	Vec2D search(/*Node, g, threshold*/);
};