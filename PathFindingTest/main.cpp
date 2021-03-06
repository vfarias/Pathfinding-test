#include <imgui.h>
#include <imgui-events-SFML.h>
#include <imgui-rendering-SFML.h>
#include <SFML/Graphics.hpp>
#include "Dijkstra.h"
#include "ThetaStar.h"
#include "HPAStar.h"
#include "IDAStar.h"
#include "Metrics.h"
#include "MapReader.h"

#define IM_ARRAYSIZE(_ARR)  ((int)(sizeof(_ARR)/sizeof(*_ARR)))

//Visual size of tiles
int tileWidth = 40;
int tileHeight = 40;
int windowWidth = 800;
int windowHeight = 600;

string* GenerateMap(int width, int height, float obstacleDensity, MapReader &mr);
void SaveDataToFile(Metrics &metrics, int chooseAlgorithm, int chooseHeuristic);
void CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid);
void CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid);
void CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, int clusterSize);
void CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid);
void CalculateDijkstra(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid);

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;

	MapReader mr = MapReader();

	//Map data
	string* map = nullptr;
	//map = mr.ReadMap("Maps/Randomized128x128-29-0.map");
	map = mr.ReadMap("Maps/maze512-1-1.map");
	//map = mr.ReadMap("Maps/adaptive-depth-1.map");
	//map = mr.ReadMap("Maps/32room_008.map");
	//map = GenerateMap(10, 10, 1.0f, mr);
	int width = mr.GetWidth();
	int height = mr.GetHeight();
	int nrOfWalls = mr.GetNrOfWalls(map);
	Vec2D startPos = {1, 1};
	Vec2D goalPos = {width-1, height-1};
	int clusterSize = 16;
	Vec2D* wallPos = new Vec2D[nrOfWalls];
	sf::RectangleShape* walls = new sf::RectangleShape[nrOfWalls];
	AStarNode** grid = nullptr;

	if (map != nullptr)
	{
		if (width > 0 && height > 0)
		{
			grid = new AStarNode*[width];

			//Initiate the grid** with walkable or non-walkable tiles
			int wallCounter = 0;
			for (int i = 0; i < width; i++)
			{
				grid[i] = new AStarNode[height];
				for (int j = 0; j < height; j++)
				{
					grid[i][j] = AStarNode(i, j);

					if (map[j*width + i] != "@")
					{
						grid[i][j]._traversable = true;
					}
					else
					{
						grid[i][j]._traversable = false;
						walls[wallCounter] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
						walls[wallCounter].setFillColor(sf::Color::White);
						walls[wallCounter].setPosition(sf::Vector2f(10.0f + (float)(tileWidth * i), 10.0f + (float)(tileHeight * j)));
						wallCounter++;
					}
				}
			}
		}
	}

	sf::View view;
	view.setCenter(0.5f * width * tileWidth, 0.5f * height * tileHeight);
	view.setSize(1.6f * width * tileWidth, 1.2f * height * tileHeight);
	sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "AI test");
	window.setFramerateLimit(60);
	window.setView(view);

	ImGui::SFML::SetRenderTarget(window);
	ImGui::SFML::InitImGuiRendering();
	ImGui::SFML::SetWindow(window);
	ImGui::SFML::InitImGuiEvents();

	sf::RectangleShape* openedTiles = nullptr;
	sf::RectangleShape* expandedTiles = nullptr;
	sf::Vertex* pathTiles = nullptr;
	
	//Mainly for the highlevel graph of HPA*
	sf::Vertex* abstractGraph = nullptr;
	sf::Vertex* openedGraph = nullptr;
	sf::Vertex* expandedGraph = nullptr;
	Metrics metrics;
	
	sf::CircleShape startNode = sf::CircleShape(0.4f*tileHeight);
	startNode.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
	startNode.setFillColor(sf::Color::Red);

	sf::CircleShape goalNode = sf::CircleShape(0.4f*tileHeight);;
	goalNode.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)tileWidth, 10.0f + goalPos._y * (float)tileHeight));
	goalNode.setFillColor(sf::Color::Yellow);

	//Other variables
	bool calculatePaths = false;
	int choosePathfinding = 0;
	int chooseHeuristic = 0;

	//Movement variable
	int delta = width * 0.05f;
	float blockSize = 32.0f;

	//Randomize map variables
	bool randomizeMap = false;
	char widthBuffer[4] = "512";
	char heightBuffer[4] = "512";
	char densityBuffer[3] = "30";

	//Set start/goal position variables
	int startOrGoal = 0;   //0 == start pos, 1 == goal pos
	char xBuffer[4] = "0";
	char yBuffer[4] = "0";

	//What should be drawn?
	bool showWalls = false;
	bool showExpandedNodes = false;
	bool showOpenedNodes = false;

	while (window.isOpen())
	{
		ImGui::SFML::UpdateImGui();
		ImGui::SFML::UpdateImGuiRendering();
		sf::Event event;
		while (window.pollEvent(event))
		{
			ImGui::SFML::ProcessEvent(event);
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}
		}

		ImGuiIO &io = ImGui::GetIO();
		//ImGui::ShowTestWindow();
		window.clear();

		/**************************************/
		/*          Start of GUI code         */
		/**************************************/
		if (ImGui::CollapsingHeader("Choose pathfinding"))
		{
			ImGui::RadioButton("A*", &choosePathfinding, 0);		ImGui::SameLine();
			ImGui::RadioButton("Theta*", &choosePathfinding, 1);	ImGui::SameLine();
			ImGui::RadioButton("HPA*", &choosePathfinding, 2);		ImGui::SameLine();
			ImGui::RadioButton("IDA*", &choosePathfinding, 3);		ImGui::SameLine();
			ImGui::RadioButton("Dijkstra", &choosePathfinding, 4);

			ImGui::RadioButton("Manhattan", &chooseHeuristic, 0);	ImGui::SameLine();
			ImGui::RadioButton("Chebyshev", &chooseHeuristic, 1);	ImGui::SameLine();
			ImGui::RadioButton("Octile", &chooseHeuristic, 2);		ImGui::SameLine();
			ImGui::RadioButton("Euclidean", &chooseHeuristic, 3);
		}
		if (ImGui::CollapsingHeader("Randomize a map"))
		{
			//Set width, height and obstacle density
			ImGui::InputText("Width", widthBuffer, IM_ARRAYSIZE(widthBuffer));
			ImGui::InputText("Height", heightBuffer, IM_ARRAYSIZE(heightBuffer));
			ImGui::InputText("Density (%)", densityBuffer, IM_ARRAYSIZE(densityBuffer));

			if (ImGui::SmallButton("Generate map"))
			{
				mr.GenerateRandomMap(stoi(string(widthBuffer)), stoi(string(heightBuffer)), 0.01f*stof(string(densityBuffer)));
			}
		}
		if (ImGui::CollapsingHeader("Set start/goal"))
		{
			ImGui::RadioButton("Set start position", &startOrGoal, 0); ImGui::SameLine();
			ImGui::RadioButton("Set goal position", &startOrGoal, 1);
			
			//Set xPos and yPos
			ImGui::InputText("X position", xBuffer, IM_ARRAYSIZE(xBuffer));
			ImGui::InputText("Y position", yBuffer, IM_ARRAYSIZE(yBuffer));

			if (ImGui::SmallButton("Set position"))
			{
				Vec2D pos = {stoi(string(xBuffer)), stoi(string(yBuffer))};
				if (startOrGoal == 0)  //Start pos
				{
					startPos = pos;
					startNode.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
				}
				else if (startOrGoal == 1)  //Goal pos
				{
					goalPos = pos;
					goalNode.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)tileWidth, 10.0f + goalPos._y * (float)tileHeight));
				}
			}
		}
		if (ImGui::CollapsingHeader("Choose what will be drawn"))
		{
			ImGui::Checkbox("Walls", &showWalls);
			ImGui::Checkbox("Opened nodes", &showOpenedNodes);
			ImGui::Checkbox("Expanded nodes", &showExpandedNodes);
		}
		if (ImGui::SmallButton("Calculate paths"))
		{
			calculatePaths = !calculatePaths;
		}

		/**************************************/
		/*            End of GUI code         */
		/**************************************/

		//Moving of the camera
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))  //Move camera west
		{
			view.setCenter(view.getCenter().x, view.getCenter().y - delta);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))  //Move camera east
		{
			view.setCenter(view.getCenter().x - delta, view.getCenter().y);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))  //Move camera south
		{
			view.setCenter(view.getCenter().x, view.getCenter().y + delta);
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))  //Move camera north
		{
			view.setCenter(view.getCenter().x + delta, view.getCenter().y);
		}

		//Zooming with the camera
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::PageUp) && blockSize <= 32.0f)  //Zoom out
		{
			view.setSize(sf::Vector2f(width * tileWidth * blockSize++ * 0.05f, height * tileHeight * blockSize++ * 0.0375f));
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::PageDown) && blockSize >= 1.0f)  //Zoom in
		{
			view.setSize(sf::Vector2f(width * tileWidth * blockSize-- * 0.05f, height * tileHeight * blockSize-- * 0.0375f));
		}
		window.setView(view);


		//Calculate pathfinding
		if (calculatePaths)
		{
			metrics.clean();
			switch (choosePathfinding)
			{
			case 0:		//A*
				CalculateAStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, width, height, startPos, goalPos, grid);
				break;
			case 1:		//Theta*
				CalculateThetaStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, width, height, startPos, goalPos, grid);
				break;
			case 2:		//HPA*
				CalculateHPAStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, width, height, startPos, goalPos, grid, clusterSize);
				break;
			case 3:		//IDA*
				CalculateIDAStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, width, height, startPos, goalPos, grid);
				break;
			case 4:
				CalculateDijkstra(metrics, (Pathfinding::Heuristic)chooseHeuristic, width, height, startPos, goalPos, grid);
			default:
				break;
			}

			if (showOpenedNodes)
			{
				if (openedTiles != nullptr)
				{
					delete[] openedTiles;
				}
				openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];

				for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
				{
					openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
					openedTiles[i].setFillColor(sf::Color(0, 200, 200, 120));
					openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
					window.draw(openedTiles[i]);
				}
			}

			if (showExpandedNodes)
			{
				if (expandedTiles != nullptr)
				{
					delete[] expandedTiles;
				}
				expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];

				for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
				{
					expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
					expandedTiles[i].setFillColor(sf::Color(200, 0, 0, 120));
					expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
					window.draw(expandedTiles[i]);
				}
			}

			if (pathTiles != nullptr)
			{
				delete[] pathTiles;
			}
			pathTiles = new sf::Vertex[metrics.getNrOfPathNodes() + 1];
			for (int i = 0; i < metrics.getNrOfPathNodes(); i++)
			{
				pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (metrics.getPathNodes()[i]._x + 0.5f), 10.0f + (float)tileHeight * (metrics.getPathNodes()[i]._y + 0.5f)));
				pathTiles[i].color = sf::Color(200, 0, 200, 255);
			}
			pathTiles[metrics.getNrOfPathNodes()] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
			
			SaveDataToFile(metrics, choosePathfinding, chooseHeuristic);
			calculatePaths = false;
		}

		//Draw the start and goal node(s)
		window.draw(startNode);
		window.draw(goalNode);

		if (choosePathfinding == 2)			//Special case for HPA*
		{
			window.draw(abstractGraph, metrics.getNrOfGraphNodes(), sf::Lines);

			if (showOpenedNodes)
			{
				window.draw(openedGraph, metrics.getNrOfOpenedNodes(), sf::Lines);
			}

			if (showExpandedNodes)
			{
				window.draw(expandedGraph, metrics.getNrOfExpandedNodes(), sf::Lines);
			}
		}
		else
		{
			if (showOpenedNodes && openedTiles != nullptr)
			{
				for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(openedTiles[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
				{
					window.draw(expandedTiles[i]);
				}
			}
		}
		window.draw(pathTiles, metrics.getNrOfPathNodes() + 1, sf::LinesStrip);

		//Draw all the walls
		if (showWalls)
		{
			for (int i = 0; i < nrOfWalls; i++)
			{
				window.draw(walls[i]);
			}
		}

		ImGui::Render();
		window.display();
	}
	delete[] expandedTiles;
	delete[] openedTiles;
	delete[] pathTiles;
	delete[] walls;
	delete[] map;
	delete[] wallPos;
	for (__int16 i = 0; i < width; i++)
	{
		delete[] grid[i];
	}
	delete[] grid;
	ImGui::SFML::Shutdown();
	return 0;
}

void SaveDataToFile(Metrics &metrics, int chooseAlgorithm, int chooseHeuristic)
{
	ofstream saveFile;
	saveFile.open("Metrics/metrics000.txt");

	//Which algorithm is used
	saveFile << "Algorithm used: ";
	switch (chooseAlgorithm)
	{
	case 0:
		saveFile << "A Star. ";
		break;

	case 1:
		saveFile << "Theta Star. ";
		break;

	case 2:
		saveFile << "HPA Star. ";
		break;

	case 3:
		saveFile << "IDA Star. ";
		break;
	default:
		saveFile << "Does not exist";
		break;
	}

	//In combination with which heuristic is being used
	saveFile << "\nHeuristic used: ";
	switch (chooseHeuristic)
	{
	case 0:
		saveFile << "Manhattan. ";
		break;

	case 1:
		saveFile << "Chebyshev. ";
		break;

	case 2:
		saveFile << "Octile. ";
		break;

	case 3:
		saveFile << "Euclidean. ";
		break;
	default:
		saveFile << "Does not exist";
		break;
	}

	//The number of opened nodes by the algorithm
	saveFile << "\nNumber of opened nodes: " << metrics.getNrOfOpenedNodes();
	saveFile << "\nNumber of expanded nodes: " << metrics.getNrOfExpandedNodes();

	//TODO L�gg till grejer h�r som metrics kan m�ta
}
string* GenerateMap(int width, int height, float obstacleDensity, MapReader &mr)
{
	string* map = nullptr;
	mr.GenerateRandomMap(width, height, obstacleDensity);

	int density = (int)(obstacleDensity*100.0f);
	stringstream ss;
	ss << "Maps/Randomized" << width << "x" << height << "-" << density << "-0.map";
	map = mr.ReadMap(ss.str());

	return map;
}
void CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid)
{
	AStar pathFinding(width, height, { 0, 0 }, grid, heuristic);
	pathFinding.init(startPos, goalPos);
	if (pathFinding.findPath(metrics))
	{}

}
void CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid)
{
	ThetaStar pathFinding(width, height, grid, heuristic);
	pathFinding.init(startPos, goalPos);
	if (pathFinding.findPath(metrics))
	{}
}
void CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, int clusterSize)
{
	HPAStar pathFinding(width, height, clusterSize, grid, heuristic);
	pathFinding.init(startPos, goalPos);
	if (pathFinding.findPath(metrics))
	{}
}
void CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid)
{
	IDAStar pathFinding(width, height, grid, heuristic);
	pathFinding.init(startPos, goalPos);
	if (pathFinding.findPath(metrics))
	{}
}
void CalculateDijkstra(Metrics &metrics, Pathfinding::Heuristic heuristic, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid)
{
	Dijkstra pathFinding(width, height, grid, heuristic);
	pathFinding.init(startPos, goalPos);
	if (pathFinding.findPath(metrics))
	{}
}