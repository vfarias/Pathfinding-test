

#include "IDAStar.h"

bool IDAStar::isAtGoal()
{
	
}
bool IDAStar::isPositionValid(Vec2D pos)
{
	return pos._x >= 0 && pos._x < _width && pos._y >= 0 && pos._y < _height;
}

IDAStar::IDAStar()
{
	_g = 0;
	_f = 0;

	_width = 0;
	_height = 0;
	_startPos = { 0,0 };
	_goalPos = { 0,0 };
}
IDAStar::IDAStar(int width, int height, Vec2D startPos, Vec2D goalPos /*Heuristic, hWeight*/)
{
	_width = width;
	_height = height;
	_startPos = startPos;
	_goalPos = goalPos;
}

IDAStar::~IDAStar()
{

}