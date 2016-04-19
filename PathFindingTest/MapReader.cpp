

#include "MapReader.h"

MapReader::MapReader()
{
	_width = 1;
	_height = 1;
	_map = nullptr;
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

	_map = new string[_width * _height];
}
MapReader::~MapReader()
{
	if (_map != nullptr)
	{
		delete[] _map;
	}
	_map = nullptr;
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
}

int MapReader::GetWidth() const
{
	return _width;
}
int MapReader::GetHeight() const
{
	return _height;
}

void MapReader::ReadMap(string fileName)
{
	stringstream ssWidth;
	stringstream ssHeight;
	ifstream file;
	string line;
	string Swidth;
	string Sheight;
	const char* CCwidth;
	const char* CCheight;
	int counter = 0;

	file.open(fileName);

	if (file.is_open())
	{
		getline(file, Swidth);
		getline(file, Sheight);

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

		_map = new string[_width * _height];

		//Read in the data from the txt-file into the _map
		while (getline(file, line))
		{
			for (int i = 0; i < _width; i++)
			{
				_map[i + counter * _width] = line.at(i);
			}
			counter++;
		}
	}
}
void MapReader::GenerateRandomMap(int width, int height, float densityOfObstacles)
{
	_width = width;
	_height = height;
	int posX = -1;
	int posY = -1;
	srand((unsigned int)time(NULL));

	_map = new string[_width * _height];

	//Initiate the array with walkable tiles
	for (int i = 0; i < _height; i++)
	{
		for (int j = 0; j < _width; j++)
		{
			_map[i * _width + j] = ".";  //All the tiles are initialized as walkable tiles
		}
	}

	//Place all the unwalkable tiles
	for (int i = 0; i < (int)(_width * _height * densityOfObstacles); i++)
	{
		posX = rand() % _width;
		posY = rand() % _height;

		if (_map[posY*_width + posX] != "@")  //The tile is walkable
		{
			_map[posY*_width + posX] = "@";  //The tile is set as unwalkable
		}
		else
		{
			posX = rand() % _width;
			posY = rand() % _height;

			while (_map[posY*_width + posX] == "@")  //Randomize again until a walkable tile is found
			{
				posX = rand() % _width;
				posY = rand() % _height;
			}

			_map[posY*_width + posX] = "@";  //The tile is set as unwalkable
		}
	}

	int density = (int)(densityOfObstacles * 100.0f);  //Converting from decimal to percent
	stringstream fileName;
	fileName << "Maps/Randomized" << to_string(_width).c_str() << "x" << to_string(_height).c_str() << "-" << to_string(density).c_str() << "-" << to_string(0) << ".txt";

	SaveMapToFile(fileName.str());
}
void MapReader::SaveMapToFile(string fileName)
{
	ofstream saveFile;
	
	saveFile.open(fileName);
	saveFile << "width " << _width << "\n";
	saveFile << "height " << _height << "\n";

	for (int i = 0; i < _height; i++)
	{
		for (int j = 0; j < _width; j++)
		{
			saveFile << _map[i * _width + j];
		}

		saveFile << "\n";
	}

	saveFile.close();
}