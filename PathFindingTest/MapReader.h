#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <time.h>
#include "Nodes.h"


#include <iostream>


/*
The map files are built up of '@', '.' and 'T'
The '@' are unwalkable
The '.' and 'T' are walkable
In this interpretation we do not care for squares that cost more than 1 (or 2^(1/2) to walk to)
*/

using namespace std;

class MapReader
{
private:
	int _width;
	int _height;
	int _nrOfWalls;

public:
	MapReader();
	MapReader(int width, int height);
	~MapReader();

	void SetWidth(int width);
	void SetHeight(int height);
	void SetMapDimensions(int width, int height);

	int GetWidth() const;
	int GetHeight() const;
	int GetNrOfWalls(string* map);

	string* ReadMap(string fileName);
	string* GenerateRandomMap(int width, int height, float densityOfObstacles);
	void SaveMapToFile(string fileName, string* map);
};