//TODO: Change order the way the map file is read in.
//Ignore the first line
//Read the second line into height
//Read the third line into width
//Ignore the fourth line
//Read the map data

#include "MapReader.h"

MapReader::MapReader()
{
	_width = 1;
	_height = 1;
	_nrOfWalls = 0;
}
MapReader::MapReader(int width, int height)
{
	//Safety measures to not get an array of size 0
	if (width > 0 && height > 0)
	{
		_width = width;
		_height = height;
	}
	else if (width <= 0)
	{
		_width = 1;
	}
	else if (height <= 0)
	{
		_height = 1;
	}
	_nrOfWalls = 0;
}
MapReader::~MapReader()
{

}

void MapReader::SetWidth(int width)
{
	//Safety measures to not get an array of size 0
	if (width < 1)
	{
		_width = 1;
	}
	else
	{
		_width = width;
	}
}
void MapReader::SetHeight(int height)
{
	//Safety measures to not get an array of size 0
	if (height < 1)
	{
		_height = 1;
	}
	else
	{
		_height = height;
	}
}
void MapReader::SetMapDimensions(int width, int height)
{
	//Safety measures to not get an array of size 0
	if (width > 0 && height > 0)
	{
		_width = width;
		_height = height;
	}
	else if (width <= 0)
	{
		_width = 1;
	}
	else if (height <= 0)
	{
		_height = 1;
	}
	_nrOfWalls = 0;
}

int MapReader::GetWidth() const
{
	return _width;
}
int MapReader::GetHeight() const
{
	return _height;
}
int MapReader::GetNrOfWalls(string* map)
{
	_nrOfWalls = 0;

	if (map != nullptr)
	{
		for (int i = 0; i < _width * _height; i++)
		{
			if (map[i] == "@")
			{
				_nrOfWalls++;
			}
		}
	}

	return _nrOfWalls;
}

string* MapReader::ReadMap(string fileName)
{
	stringstream ssHeight;
	stringstream ssWidth;
	ifstream file;
	string line;
	string trash;  //Will store the values that aren't needed
	string Swidth;
	string Sheight;
	string* map = nullptr;
	const char* CCheight;
	const char* CCwidth;
	int counter = 0;

	file.open(fileName);

	if (file.is_open())
	{
		getline(file, trash);  //Ignore this line
		getline(file, Sheight);
		getline(file, Swidth);
		getline(file, trash);  //Ignore this line
		//Convert the strings to const char*
		CCwidth = Swidth.c_str();
		CCheight = Sheight.c_str();

		//Save the const char* as ints
		ssWidth << CCwidth;
		ssWidth.seekg(6);  // 6 since 'width ' is 6 letters, note the space
		ssWidth >> _width;

		//Save the const char* as ints
		ssHeight << CCheight;
		ssHeight.seekg(7);  // 7 since 'height ' is 7 letters, note the space
		ssHeight >> _height;

		map = new string[_width * _height];

		//Read in the data from the txt-file into the map
		while (getline(file, line))
		{
			for (int i = 0; i < _width; i++)
			{
				map[i + counter * _width] = line.at(i);
			}
			counter++;
		}
	}

	return map;
}
string* MapReader::GenerateRandomMap(int width, int height, float densityOfObstacles)
{
	_width = width;
	_height = height;
	int posX = -1;
	int posY = -1;
	srand((unsigned int)time(NULL));

	string* map = new string[_width * _height];

	//Initiate the array with walkable tiles
	for (int i = 0; i < _height; i++)
	{
		for (int j = 0; j < _width; j++)
		{
			map[i * _width + j] = ".";  //All the tiles are initialized as walkable tiles
		}
	}

	//Place all the unwalkable tiles
	for (int i = 0; i < (int)(_width * _height * densityOfObstacles); i++)
	{
		posX = rand() % _width;
		posY = rand() % _height;

		if (map[posY*_width + posX] != "@")  //The tile is walkable
		{
			map[posY*_width + posX] = "@";  //The tile is set as unwalkable
		}
		else
		{
			while (map[posY*_width + posX] == "@")  //Randomize again until a walkable tile is found
			{
				posX = rand() % _width;
				posY = rand() % _height;
			}

			map[posY*_width + posX] = "@";  //The tile is set as unwalkable
		}
	}

	int density = (int)(densityOfObstacles * 100.0f);  //Converting from decimal to percent
	stringstream fileName;
	fileName << "Maps/Randomized" << to_string(_width).c_str() << "x" << to_string(_height).c_str() << "-" << to_string(density).c_str() << "-" << to_string(0) << ".map";

	SaveMapToFile(fileName.str(), map);
	return map;
}
void MapReader::SaveMapToFile(string fileName, string* map)
{
	ofstream saveFile;
	
	saveFile.open(fileName);

	saveFile << "type heuristic\n";
	saveFile << "height " << _height << "\n";
	saveFile << "width " << _width << "\n";
	saveFile << "unnecessary line of text\n";

	for (int i = 0; i < _height; i++)
	{
		for (int j = 0; j < _width; j++)
		{
			saveFile << map[i * _width + j];
		}

		saveFile << "\n";
	}

	saveFile.close();
}