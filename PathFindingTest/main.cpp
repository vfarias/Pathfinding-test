#include <imgui.h>
#include <imgui-events-SFML.h>
#include <imgui-rendering-SFML.h>
#include <SFML/Graphics.hpp>
#include "AStar.h"
#include "ThetaStar.h"
#include "HPAStar.h"
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
void CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* &pathTiles, sf::RectangleShape* &openedTiles, sf::RectangleShape* &expandedTiles, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, Vec2D* &path, int &pathLength);
void CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
void CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
void CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;

	//AI variables
	int AStar_pathLength = 0;
	int ThetaStar_pathLength = 0;
	int HPAStar_pathLength = 0;
	int IDAStar_pathLength = 0;

	MapReader mr = MapReader();

	Vec2D startPos = {0,0};
	Vec2D goalPos = {1,1};
	Vec2D* path = nullptr;

	//Map data
	string* map = nullptr;
	//map = mr.ReadMap("Maps/Randomized10x10-10-0.map");
	//map = mr.ReadMap("Maps/maze512-1-1.map");
	map = mr.ReadMap("Maps/adaptive-depth-1.map");
	//map = GenerateMap(10, 10, 1.0f, mr);
	int width = mr.GetWidth();
	int height = mr.GetHeight();
	int nrOfWalls = mr.GetNrOfWalls(map);
	int clusterSize = 32;
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

	//A*
	sf::RectangleShape* AStar_openedTiles = nullptr;
	sf::RectangleShape* AStar_expandedTiles = nullptr;
	sf::Vertex* AStar_pathTiles = nullptr;

	//Theta*
	sf::RectangleShape* ThetaStar_openedTiles = nullptr;
	sf::RectangleShape* ThetaStar_expandedTiles = nullptr;
	sf::Vertex* ThetaStar_pathTiles = nullptr;

	//HPA*
	sf::Vertex* HPAabstractGraph = nullptr;
	sf::Vertex* HPAopenedGraph = nullptr;
	sf::Vertex* HPAexpandedGraph = nullptr;

	//IDA*
	sf::RectangleShape* IDAStar_openedTiles = nullptr;
	sf::RectangleShape* IDAStar_expandedTiles = nullptr;
	sf::Vertex* IDAStar_pathTiles = nullptr;

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
	int delta = 10;
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
		if (ImGui::BeginMenu("Choose pathfinding"))
		{
			ImGui::RadioButton("A*", &choosePathfinding, 0);		ImGui::SameLine();
			ImGui::RadioButton("Theta*", &choosePathfinding, 1);	ImGui::SameLine();
			ImGui::RadioButton("HPA*", &choosePathfinding, 2);		ImGui::SameLine();
			ImGui::RadioButton("IDA*", &choosePathfinding, 3);

			ImGui::RadioButton("Manhattan", &chooseHeuristic, 0);	ImGui::SameLine();
			ImGui::RadioButton("Chebyshev", &chooseHeuristic, 1);	ImGui::SameLine();
			ImGui::RadioButton("Octile", &chooseHeuristic, 2);		ImGui::SameLine();
			ImGui::RadioButton("Euclidean", &chooseHeuristic, 3);

			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Randomize a map"))
		{
			//Set width, height and obstacle density
			ImGui::InputText("Width", widthBuffer, IM_ARRAYSIZE(widthBuffer));
			ImGui::InputText("Height", heightBuffer, IM_ARRAYSIZE(heightBuffer));
			ImGui::InputText("Density (%)", densityBuffer, IM_ARRAYSIZE(densityBuffer));

			if (ImGui::SmallButton("Generate map"))
			{
				mr.GenerateRandomMap(stoi(string(widthBuffer)), stoi(string(heightBuffer)), 0.01f*stof(string(densityBuffer)));
			}

			ImGui::EndMenu();
		}
		if (ImGui::CollapsingHeader("Set start/goal positions"))
		{
			ImGui::RadioButton("Set start position", &startOrGoal, 0); ImGui::SameLine();
			ImGui::RadioButton("Set goal position", &startOrGoal, 1);

			//Set xPos and yPos
			ImGui::InputText("X position", xBuffer, IM_ARRAYSIZE(xBuffer));
			ImGui::InputText("Y position", yBuffer, IM_ARRAYSIZE(yBuffer));

			Vec2D pos = {stoi(string(xBuffer)), stoi(string(yBuffer))};

			if (ImGui::SmallButton("Set position"))
			{
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
		if (ImGui::BeginMenu("Choose what will be drawn"))
		{
			ImGui::MenuItem("Walls", NULL, &showWalls);
			ImGui::MenuItem("Opened nodes", NULL, &showOpenedNodes);
			ImGui::MenuItem("Expanded nodes", NULL, &showExpandedNodes);

			ImGui::EndMenu();
		}
		ImGui::MenuItem("Calculate paths", NULL, &calculatePaths);

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
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::PageUp))  //Zoom out
		{
			view.setSize(sf::Vector2f(width * tileWidth * blockSize++ * 0.05f, height * tileHeight * blockSize++ * 0.0375f));
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::PageDown))  //Zoom in
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
				CalculateAStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos, goalPos, grid, path, AStar_pathLength);
				break;
			case 1:		//Theta*
				CalculateThetaStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
				break;
			case 2:		//HPA*
				//CalculateHPAStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
				break;
			case 3:		//IDA*
				CalculateIDAStar(metrics, (Pathfinding::Heuristic)chooseHeuristic, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
				break;
			default:
				break;
			}
			SaveDataToFile(metrics, choosePathfinding, chooseHeuristic);
			calculatePaths = false;
		}

		//Draw the start and goal node(s)
		window.draw(startNode);
		window.draw(goalNode);
		
		if (choosePathfinding == 0)
		{
			window.draw(AStar_pathTiles, AStar_pathLength + 1, sf::LinesStrip);
			if (showOpenedNodes)
			{
				for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(AStar_openedTiles[i]);
				}
			}
			if (showExpandedNodes)
			{
				for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
				{
					window.draw(AStar_expandedTiles[i]);
				}
			}
		}
		if (choosePathfinding == 1)
		{
			window.draw(ThetaStar_pathTiles, ThetaStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(ThetaStar_openedTiles[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
				{
					window.draw(ThetaStar_expandedTiles[i]);
				}
			}
		}
		if (choosePathfinding == 2)
		{
			window.draw(HPAabstractGraph, HPAStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
				{
					//window.draw(HPAopenedGraph[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
				{
					//window.draw(HPAexpandedGraph[i]);
				}
			}
		}
		if (choosePathfinding == 3)
		{
			window.draw(IDAStar_pathTiles, IDAStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(IDAStar_openedTiles[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
				{
					window.draw(IDAStar_expandedTiles[i]);
				}
			}
		}

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

	//pathFinding.cleanMap();
	delete[] HPAexpandedGraph;
	delete[] HPAopenedGraph;
	delete[] HPAabstractGraph;

	delete[] AStar_expandedTiles;
	delete[] ThetaStar_expandedTiles;
	delete[] HPAexpandedGraph;
	delete[] IDAStar_expandedTiles;

	delete[] AStar_openedTiles;
	delete[] ThetaStar_openedTiles;
	delete[] HPAopenedGraph;
	delete[] IDAStar_openedTiles;

	delete[] AStar_pathTiles;
	delete[] ThetaStar_pathTiles;
	delete[] HPAabstractGraph;
	delete[] IDAStar_pathTiles;

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

	//TODO Lägg till grejer här som metrics kan mäta
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
void CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* &pathTiles, sf::RectangleShape* &openedTiles, sf::RectangleShape* &expandedTiles, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, Vec2D* &path, int &pathLength)
{
	AStar pathFinding(width, height, { 0, 0 }, startPos, goalPos, grid, heuristic);
	pathFinding.init(startPos, goalPos);

	if (pathTiles != nullptr)
	{
		delete[] pathTiles;
		pathTiles = nullptr;
	}
	if (openedTiles != nullptr)
	{
		delete[] openedTiles;
		openedTiles = nullptr;
	}
	if (expandedTiles != nullptr)
	{
		delete[] expandedTiles;
		expandedTiles = nullptr;
	}

	if (pathFinding.findPath(metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}

	openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];

	for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		openedTiles[i].setFillColor(sf::Color(0, 200, 200, 120));
		openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
	}

	expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];

	for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles[i].setFillColor(sf::Color(200, 0, 0, 120));
		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
	}

	pathTiles = new sf::Vertex[pathLength + 1];

	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 0, 200, 255);
	}

	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
}
void CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	//ThetaStar pathFinding2(width, height, startPos, goalPos, heuristic);
	//pathFinding2.init(startPos, goalPos);

	//if (pathTiles != nullptr)
	//{
	//	delete[] pathTiles;
	//	pathTiles = nullptr;
	//}
	//if (openedTiles != nullptr)
	//{
	//	delete[] openedTiles;
	//	openedTiles = nullptr;
	//}
	//if (expandedTiles != nullptr)
	//{
	//	delete[] expandedTiles;
	//	expandedTiles = nullptr;
	//}

	//AStar pathFinding(width, height, startPos, goalPos, heuristic);

	//if (pathFinding.findPath(metrics))
	//{
	//	pathLength = pathFinding.getNrOfPathNodes();
	//	path = pathFinding.getPath();
	//}
	//openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
	//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	//{
	//	openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
	//	openedTiles[i].setFillColor(sf::Color(0, 0, 200, 120));
	//	openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
	//}
	//expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
	//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	//{
	//	expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
	//	expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
	//	expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
	//}
	//pathTiles = new sf::Vertex[pathLength + 1];
	//for (int i = 0; i < pathLength; i++)
	//{
	//	pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
	//	pathTiles[i].color = sf::Color(200, 200, 0, 255);
	//}
	//pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
}
void CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
	//HPAStar pathFinding3(width, height, clusterSize, Pathfinding::OCTILE);
	//pathFinding3.init(startPos, goalPos);

	//if (pathTiles != nullptr)
	//{
	//	delete[] pathTiles;
	//	pathTiles = nullptr;
	//}
	//if (openedTiles != nullptr)
	//{
	//	delete[] openedTiles;
	//	openedTiles = nullptr;
	//}
	//if (expandedTiles != nullptr)
	//{
	//	delete[] expandedTiles;
	//	expandedTiles = nullptr;
	//}

	//
}
void CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	//if (pathTiles != nullptr)
	//{
	//	delete[] pathTiles;
	//	pathTiles = nullptr;
	//}
	//if (openedTiles != nullptr)
	//{
	//	delete[] openedTiles;
	//	openedTiles = nullptr;
	//}
	//if (expandedTiles != nullptr)
	//{
	//	delete[] expandedTiles;
	//	expandedTiles = nullptr;
	//}

	/**/
}