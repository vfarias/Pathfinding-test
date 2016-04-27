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
int tileWidth = 10;
int tileHeight = 10;

string* GenerateMap(int width, int height, float obstacleDensity, MapReader &mr);
bool CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* &pathTiles, sf::RectangleShape* &openedTiles, sf::RectangleShape* &expandedTiles, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, Vec2D* &path, int &pathLength);
bool CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);

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
	static Vec2D startPos  = {0,0};
	static Vec2D startPos2 = {0,2};
	static Vec2D startPos3 = {2,0};
	static Vec2D startPos4 = {2,2};
	Vec2D goalPos  = {3, 0};
	Vec2D goalPos2 = {3, 2};
	Vec2D goalPos3 = {5, 0};
	Vec2D goalPos4 = {5, 2};
	Vec2D* path = nullptr;

	//Map data
	string* map = nullptr;
	map = mr.ReadMap("Maps/Randomized10x10-10-0.map");
	//map = mr.ReadMap("Maps/maze512-1-1.map");
	//map = GenerateMap(10, 10, 0.0f, mr);
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
			for (int i = 0; i < width; i++)
			{
				grid[i] = new AStarNode[height];
				for (int j = 0; j < height; j++)
				{
					grid[i][j] = AStarNode(i, j);

					if (map[i*width + j] != "@")
					{
						grid[i][j]._traversable = true;
					}
					else
					{
						grid[i][j]._traversable = false;
					}
				}
			}

			//Place the walls in an array of its own (easier to traverse through)
			int counter = 0;
			for (int i = 0; i < width * height; i++)
			{
				if (!grid[(i / width)][(i % width)]._traversable)
				{
					walls[counter] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
					walls[counter].setFillColor(sf::Color::White);
					walls[counter].setPosition(sf::Vector2f(10.0f + (float)tileWidth * (i%width), 10.0f + (float)tileHeight * (i / width)));
					counter++;
				}
			}
		}
	}

	sf::RenderWindow window(sf::VideoMode(800, 600), "AI test");
	window.setFramerateLimit(60);

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

/////////////////////////////////////////////////////////////////////////////

	/*
	expandedTiles = new sf::RectangleShape[ThetaStar_metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < ThetaStar_metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * ThetaStar_metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * ThetaStar_metrics.getExpandedNodes()[i]._y));
	}
	abstractGraph = new sf::Vertex[ThetaStar_metrics.getNrOfGraphNodes()];
	for (int i = 0; i < ThetaStar_metrics.getNrOfGraphNodes(); i++)
	{
		abstractGraph[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (ThetaStar_metrics.getGraphNodes()[i]._x + 0.5f), 10.0f + (float)tileHeight * (ThetaStar_metrics.getGraphNodes()[i]._y + 0.5f)));
		abstractGraph[i].color = sf::Color(200, 0, 0, 255);
	}
	expandedGraph = new sf::Vertex[ThetaStar_metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < ThetaStar_metrics.getNrOfExpandedNodes(); i++)
	{
		expandedGraph[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (ThetaStar_metrics.getExpandedNodes()[i]._x + 0.5f), 10.0f + (float)tileHeight * (ThetaStar_metrics.getExpandedNodes()[i]._y + 0.5f)));
		expandedGraph[i].color = sf::Color(200, 200, 0, 255);
	}
	openedGraph = new sf::Vertex[ThetaStar_metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < ThetaStar_metrics.getNrOfOpenedNodes(); i++)
	{
		openedGraph[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (ThetaStar_metrics.getOpenedNodes()[i]._x + 0.5f), 10.0f + (float)tileHeight * (ThetaStar_metrics.getOpenedNodes()[i]._y + 0.5f)));
		openedGraph[i].color = sf::Color(50, 50, 250, 255);
	}
	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
	*/
	/*****************************************************************************/
	/*
	openedTiles2 = new sf::RectangleShape[AStar_metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < AStar_metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles2[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		openedTiles2[i].setFillColor(sf::Color(0, 0, 200, 120));
		openedTiles2[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * AStar_metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * AStar_metrics.getOpenedNodes()[i]._y));
	}
	expandedTiles2 = new sf::RectangleShape[AStar_metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < AStar_metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles2[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles2[i].setFillColor(sf::Color(0, 200, 0, 120));
		expandedTiles2[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * AStar_metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * AStar_metrics.getExpandedNodes()[i]._y));
	}
	pathTiles2 = new sf::Vertex[pathLength2 + 1];
	for (int i = 0; i < pathLength2; i++)
	{
		pathTiles2[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
		pathTiles2[i].color = sf::Color(200, 0, 200, 255);
	}
	pathTiles2[pathLength2] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
	*/

	Metrics AStar_metrics = Metrics();
	Metrics ThetaStar_metrics = Metrics();
	Metrics HPAStar_metrics = Metrics();
	Metrics IDAStar_metrics = Metrics();

	sf::CircleShape AStar_start(0.4f * tileHeight);
	sf::CircleShape ThetaStar_start(0.4f * tileHeight);
	sf::CircleShape HPAStar_start(0.4f * tileHeight);
	sf::CircleShape IDAStar_start(0.4f * tileHeight);

	sf::CircleShape AStar_goal(0.4f * tileHeight);
	sf::CircleShape ThetaStar_goal(0.4f * tileHeight);
	sf::CircleShape HPAStar_goal(0.4f * tileHeight);
	sf::CircleShape IDAStar_goal(0.4f * tileHeight);

	AStar_start.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
	AStar_start.setFillColor(sf::Color::Red);
	ThetaStar_start.setPosition(sf::Vector2f(10.0f + startPos2._x * (float)tileWidth, 10.0f + startPos2._y * (float)tileHeight));
	ThetaStar_start.setFillColor(sf::Color::Blue);
	HPAStar_start.setPosition(sf::Vector2f(10.0f + startPos3._x * (float)tileWidth, 10.0f + startPos3._y * (float)tileHeight));
	HPAStar_start.setFillColor(sf::Color::Green);
	IDAStar_start.setPosition(sf::Vector2f(10.0f + startPos4._x * (float)tileWidth, 10.0f + startPos4._y * (float)tileHeight));
	IDAStar_start.setFillColor(sf::Color::Magenta);

	AStar_goal.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)tileWidth, 10.0f + goalPos._y * (float)tileHeight));
	AStar_goal.setFillColor(sf::Color::Yellow);
	ThetaStar_goal.setPosition(sf::Vector2f(10.0f + goalPos2._x * (float)tileWidth, 10.0f + goalPos2._y * (float)tileHeight));
	ThetaStar_goal.setFillColor(sf::Color::Yellow);
	HPAStar_goal.setPosition(sf::Vector2f(10.0f + goalPos3._x * (float)tileWidth, 10.0f + goalPos3._y * (float)tileHeight));
	HPAStar_goal.setFillColor(sf::Color::Yellow);
	IDAStar_goal.setPosition(sf::Vector2f(10.0f + goalPos4._x * (float)tileWidth, 10.0f + goalPos4._y * (float)tileHeight));
	IDAStar_goal.setFillColor(sf::Color::Yellow);

	//Different pathfinding and heuristic combinations
	static bool AStarManhattan = false;
	static bool AStarChebyshev = false;
	static bool AStarOctile = false;
	static bool AStarEuclidean = false;

	static bool ThetaStarManhattan = false;
	static bool ThetaStarChebyshev = false;
	static bool ThetaStarOctile = false;
	static bool ThetaStarEuclidean = false;

	static bool IDAStarManhattan = false;
	static bool IDAStarChebyshev = false;
	static bool IDAStarOctile = false;
	static bool IDAStarEuclidean = false;

	static bool HPAStarManhattan = false;
	static bool HPAStarChebyshev = false;
	static bool HPAStarOctile = false;
	static bool HPAStarEuclidean = false;

	static bool GAAStarManhattan = false;
	static bool GAAStarChebyshev = false;
	static bool GAAStarOctile = false;
	static bool GAAStarEuclidean = false;

	static bool DStarLiteManhattan = false;
	static bool DStarLiteChebyshev = false;
	static bool DStarLiteOctile = false;
	static bool DStarLiteEuclidean = false;

	//Other variables
	bool removePathFinding = false;

	//Randomize map variables
	bool randomizeMap = false;
	static char widthBuffer[4] = "512";
	static char heightBuffer[4] = "512";
	static char densityBuffer[3] = "30";

	//Set start/goal position variables
	static int startOrGoal = 0;   //0 == start pos, 1 == goal pos
	static int SetPosition = 0;   //0 == A*, 1 == Theta*, 2 == HPA*, 3 == IDA*
	static int SetGoal = 0;       //0 == A*, 1 == Theta*, 2 == HPA*, 3 == IDA*
	static char xBuffer[4] = "0";
	static char yBuffer[4] = "0";

	//What should be drawn?
	static bool showWalls = false;
	static bool showExpandedNodes = false;
	static bool showOpenedNodes = false;

	bool calculatePaths = false;

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
			//TODO: Add interaction with the file system
			if (ImGui::BeginMenu("A*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &AStarManhattan);
				ImGui::MenuItem("Chebyshev", NULL, &AStarChebyshev);
				ImGui::MenuItem("Octile", NULL, &AStarOctile);
				ImGui::MenuItem("Euclidean", NULL, &AStarEuclidean);
				ImGui::EndMenu();

				//TODO: Read the A* path finding with the heuristic chosen
			}
			if (ImGui::BeginMenu("Theta*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &ThetaStarManhattan);
				ImGui::MenuItem("Chebyshev", NULL, &ThetaStarChebyshev);
				ImGui::MenuItem("Octile", NULL, &ThetaStarOctile);
				ImGui::MenuItem("Euclidean", NULL, &ThetaStarEuclidean);
				ImGui::EndMenu();

				//TODO: Read the Theta* path finding
			}
			if (ImGui::BeginMenu("HPA*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &HPAStarManhattan);
				ImGui::MenuItem("Chebyshev", NULL, &HPAStarChebyshev);
				ImGui::MenuItem("Octile", NULL, &HPAStarOctile);
				ImGui::MenuItem("Euclidean", NULL, &HPAStarEuclidean);
				ImGui::EndMenu();

				//TODO: Read the HPA* path finding
			}
			if (ImGui::BeginMenu("IDA*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &IDAStarManhattan);
				ImGui::MenuItem("Chebyshev", NULL, &IDAStarChebyshev);
				ImGui::MenuItem("Octile", NULL, &IDAStarOctile);
				ImGui::MenuItem("Euclidean", NULL, &IDAStarEuclidean);
				ImGui::EndMenu();

				//TODO: Read the IDA* path finding
			}

			ImGui::EndMenu();
		}
		if (calculatePaths && (AStarManhattan || AStarChebyshev || AStarOctile || AStarEuclidean))  //A*
		{
			if (AStarManhattan)
			{
				CalculateAStar(AStar_metrics, Pathfinding::MANHATTAN, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos, goalPos, grid, path, AStar_pathLength);
			}
			else if (AStarChebyshev)
			{
				CalculateAStar(AStar_metrics, Pathfinding::CHEBYSHEV, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos, goalPos, grid, path, AStar_pathLength);
			}
			else if (AStarOctile)
			{
				CalculateAStar(AStar_metrics, Pathfinding::OCTILE, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos, goalPos, grid, path, AStar_pathLength);
			}
			else if (AStarEuclidean)
			{
				CalculateAStar(AStar_metrics, Pathfinding::EUCLIDEAN, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos, goalPos, grid, path, AStar_pathLength);
			}
		}
		if (calculatePaths && (ThetaStarManhattan || ThetaStarChebyshev || ThetaStarOctile || ThetaStarEuclidean))  //Theta*
		{
			if (ThetaStarManhattan)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::MANHATTAN, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
			else if (ThetaStarChebyshev)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::CHEBYSHEV, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
			else if (ThetaStarOctile)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::OCTILE, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
			else if (ThetaStarEuclidean)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::EUCLIDEAN, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
		}
		if (calculatePaths && (HPAStarManhattan || HPAStarChebyshev || HPAStarOctile || HPAStarEuclidean))  //HPA*
		{
			//if (HPAStarManhattan)
			//{
			//	CalculateHPAStar(HPAStar_metrics, Pathfinding::MANHATTAN, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//else if (HPAStarChebyshev)
			//{
			//	CalculateHPAStar(HPAStar_metrics, Pathfinding::CHEBYSHEV, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//else if (HPAStarOctile)
			//{
			//	CalculateHPAStar(HPAStar_metrics, Pathfinding::OCTILE, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//else if (HPAStarEuclidean)
			//{
			//	CalculateHPAStar(HPAStar_metrics, Pathfinding::EUCLIDEAN, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
		}
		if (calculatePaths && (IDAStarManhattan || IDAStarChebyshev || IDAStarOctile || IDAStarEuclidean))  //IDA*
		{
			if (IDAStarManhattan)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::MANHATTAN, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
			else if (IDAStarChebyshev)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::CHEBYSHEV, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
			else if (IDAStarOctile)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::OCTILE, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
			else if (IDAStarEuclidean)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::EUCLIDEAN, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
		}
		ImGui::MenuItem("Remove all pathfinding", NULL, &removePathFinding);
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
		if (ImGui::BeginMenu("Set start position"))
		{
			ImGui::RadioButton("Set start position", &startOrGoal, 0); ImGui::SameLine();
			ImGui::RadioButton("Set goal position", &startOrGoal, 1);

			ImGui::RadioButton("A*", &SetPosition, 0);			ImGui::SameLine();
			ImGui::RadioButton("Theta*", &SetPosition, 1);		ImGui::SameLine();
			ImGui::RadioButton("HPA*", &SetPosition, 2);		ImGui::SameLine();
			ImGui::RadioButton("IDA*", &SetPosition, 3);

			ImGui::RadioButton("Manhattan", &SetGoal, 0);		ImGui::SameLine();
			ImGui::RadioButton("Chebyshev", &SetGoal, 1);		ImGui::SameLine();
			ImGui::RadioButton("Octile", &SetGoal, 2);			ImGui::SameLine();
			ImGui::RadioButton("Euclidean", &SetGoal, 3);

			//Set xPos and yPos
			ImGui::InputText("Y position", xBuffer, IM_ARRAYSIZE(xBuffer));
			ImGui::InputText("X position", yBuffer, IM_ARRAYSIZE(yBuffer));

			Vec2D pos = {stoi(string(xBuffer)), stoi(string(yBuffer))};

			if (ImGui::SmallButton("Set position"))
			{
				if (startOrGoal == 0)  //Start pos
				{
					if (SetPosition == 0)
					{
						startPos = pos;
						AStar_start.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
					}
					if (SetPosition == 1)
					{
						startPos2 = pos;
						ThetaStar_start.setPosition(sf::Vector2f(10.0f + startPos2._x * (float)tileWidth, 10.0f + startPos2._y * (float)tileHeight));
					}
					if (SetPosition == 2)
					{
						startPos3 = pos;
						HPAStar_start.setPosition(sf::Vector2f(10.0f + startPos3._x * (float)tileWidth, 10.0f + startPos3._y * (float)tileHeight));
					}
					if (SetPosition == 3)
					{
						startPos4 = pos;
						IDAStar_start.setPosition(sf::Vector2f(10.0f + startPos4._x * (float)tileWidth, 10.0f + startPos4._y * (float)tileHeight));
					}
				}
				else if (startOrGoal == 1)  //Goal pos
				{
					if (SetGoal == 0)
					{
						goalPos = pos;
						AStar_goal.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)tileWidth, 10.0f + goalPos._y * (float)tileHeight));
					}
					if (SetGoal == 1)
					{
						goalPos2 = pos;
						ThetaStar_goal.setPosition(sf::Vector2f(10.0f + goalPos2._x * (float)tileWidth, 10.0f + goalPos2._y * (float)tileHeight));
					}
					if (SetGoal == 2)
					{
						goalPos3 = pos;
						HPAStar_goal.setPosition(sf::Vector2f(10.0f + goalPos3._x * (float)tileWidth, 10.0f + goalPos3._y * (float)tileHeight));
					}
					if (SetGoal == 3)
					{
						goalPos4 = pos;
						IDAStar_goal.setPosition(sf::Vector2f(10.0f + goalPos4._x * (float)tileWidth, 10.0f + goalPos4._y * (float)tileHeight));
					}
				}
			}

			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Choose what will be drawn"))
		{
			ImGui::MenuItem("Walls", NULL, &showWalls);
			ImGui::MenuItem("Opened nodes", NULL, &showOpenedNodes);
			ImGui::MenuItem("Expanded nodes", NULL, &showExpandedNodes);

			ImGui::EndMenu();
		}
		ImGui::MenuItem("Calculate paths", NULL, &calculatePaths);
		
		if (removePathFinding)
		{
			AStarManhattan = false;
			AStarChebyshev = false;
			AStarOctile = false;
			AStarEuclidean = false;

			ThetaStarManhattan = false;
			ThetaStarChebyshev = false;
			ThetaStarOctile = false;
			ThetaStarEuclidean = false;

			IDAStarManhattan = false;
			IDAStarChebyshev = false;
			IDAStarOctile = false;
			IDAStarEuclidean = false;

			HPAStarManhattan = false;
			HPAStarChebyshev = false;
			HPAStarOctile = false;
			HPAStarEuclidean = false;

			GAAStarManhattan = false;
			GAAStarChebyshev = false;
			GAAStarOctile = false;
			GAAStarEuclidean = false;

			DStarLiteManhattan = false;
			DStarLiteChebyshev = false;
			DStarLiteOctile = false;
			DStarLiteEuclidean = false;

			removePathFinding = false;
		}

		/**************************************/
		/*            End of GUI code         */
		/**************************************/

		//Draw the start and goal node(s)
		if (AStarManhattan || AStarChebyshev || AStarOctile || AStarEuclidean)
		{
			window.draw(AStar_start);
			window.draw(AStar_goal);
			window.draw(AStar_pathTiles, AStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < AStar_metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(AStar_openedTiles[i]);
				}
			}
			if (showExpandedNodes)
			{
				for (int i = 0; i < AStar_metrics.getNrOfExpandedNodes(); i++)
				{
					window.draw(AStar_expandedTiles[i]);
				}
			}
		}
		if (ThetaStarManhattan || ThetaStarChebyshev || ThetaStarOctile || ThetaStarEuclidean)
		{
			window.draw(ThetaStar_start);
			window.draw(ThetaStar_goal);
			window.draw(ThetaStar_pathTiles, ThetaStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < ThetaStar_metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(ThetaStar_openedTiles[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < ThetaStar_metrics.getNrOfExpandedNodes(); i++)
				{
					window.draw(ThetaStar_expandedTiles[i]);
				}
			}
		}
		if (HPAStarManhattan || HPAStarChebyshev || HPAStarOctile || HPAStarEuclidean)
		{
			window.draw(HPAStar_start);
			window.draw(HPAStar_goal);
			window.draw(HPAabstractGraph, HPAStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < HPAStar_metrics.getNrOfOpenedNodes(); i++)
				{
					//window.draw(HPAopenedGraph[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < HPAStar_metrics.getNrOfExpandedNodes(); i++)
				{
					//window.draw(HPAexpandedGraph[i]);
				}
			}
		}
		if (IDAStarManhattan || IDAStarChebyshev || IDAStarOctile || IDAStarEuclidean)
		{
			window.draw(IDAStar_start);
			window.draw(IDAStar_goal);
			window.draw(IDAStar_pathTiles, IDAStar_pathLength + 1, sf::LinesStrip);

			if (showOpenedNodes)
			{
				for (int i = 0; i < IDAStar_metrics.getNrOfOpenedNodes(); i++)
				{
					window.draw(IDAStar_openedTiles[i]);
				}
			}

			if (showExpandedNodes)
			{
				for (int i = 0; i < IDAStar_metrics.getNrOfExpandedNodes(); i++)
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
bool CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* &pathTiles, sf::RectangleShape* &openedTiles, sf::RectangleShape* &expandedTiles, int width, int height, Vec2D startPos, Vec2D goalPos, AStarNode** grid, Vec2D* &path, int &pathLength)
{
	AStar pathFinding(width, height, { 0, 0 }, startPos, goalPos, grid, heuristic);
	pathFinding.init(startPos, goalPos);

	for (int i = 0; i < width*height; i++)
	{
		pathFinding.setTraversable({ (i % width), (i / width) }, grid[(i%width)][(i / width)]._traversable);  //Initiate the pathfinding's map
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
	
	return false;
}
bool CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	//ThetaStar pathFinding2(width, height, startPos, goalPos, heuristic);
	//pathFinding2.init(startPos, goalPos);

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

	return false;
}
bool CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
	//HPAStar pathFinding3(width, height, clusterSize, Pathfinding::OCTILE);
	//pathFinding3.init(startPos, goalPos);

	//
	return false;
}
bool CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	/**/
	return false;
}