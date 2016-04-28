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

const int NR_OF_ALGORITHMS = 4;
const int NR_OF_HEURISTICS = 4;

string* GenerateMap(int width, int height, float obstacleDensity, MapReader &mr);
void SaveDataToFile(Metrics metrics, bool algorithmUsed[NR_OF_ALGORITHMS], bool heuristicUsed[NR_OF_HEURISTICS]);
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

	Vec2D startPos[NR_OF_ALGORITHMS];
	Vec2D goalPos[NR_OF_ALGORITHMS];
	startPos[0] = {0, 0};
	startPos[1] = {0, 2};
	startPos[2] = {2, 0};
	startPos[3] = {2, 2};
	goalPos[0]  = {4, 0};
	goalPos[1]  = {4, 2};
	goalPos[2]  = {6, 0};
	goalPos[3]  = {6, 2};
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

	bool algorithmCombinations[NR_OF_ALGORITHMS * NR_OF_HEURISTICS];
	for (int i = 0; i < NR_OF_ALGORITHMS * NR_OF_HEURISTICS; i++)
	{
		algorithmCombinations[i] = false;
	}
	/**************************************
	--Order of the algorithms:
	AStarManhattan
	AStarChebyshev
	AStarOctile
	AStarEuclidean

	ThetaStarManhattan
	ThetaStarChebyshev
	ThetaStarOctile
	ThetaStarEuclidean

	IDAStarManhattan
	IDAStarChebyshev
	IDAStarOctile
	IDAStarEuclidean

	HPAStarManhattan
	HPAStarChebyshev
	HPAStarOctile
	HPAStarEuclidean
	**************************************/

	bool algorithmUsed[NR_OF_ALGORITHMS];
	for (int i = 0; i < NR_OF_ALGORITHMS; i++)
	{
		algorithmUsed[i] = false;
	}
	bool heuristicUsed[NR_OF_HEURISTICS];
	for (int i = 0; i < NR_OF_HEURISTICS; i++)
	{
		heuristicUsed[i] = false;
	}

	Metrics metrics;
	
	sf::CircleShape startNodes[NR_OF_ALGORITHMS];
	for (int i = 0; i < NR_OF_ALGORITHMS; i++)
	{
		startNodes[i] = sf::CircleShape(0.4f*tileHeight);
		startNodes[i].setPosition(sf::Vector2f(10.0f + startPos[i]._x * (float)tileWidth, 10.0f + startPos[i]._y * (float)tileHeight));
	}
	startNodes[0].setFillColor(sf::Color::Red);
	startNodes[1].setFillColor(sf::Color::Blue);
	startNodes[2].setFillColor(sf::Color::Green);
	startNodes[3].setFillColor(sf::Color::Magenta);

	sf::CircleShape goalNodes[NR_OF_ALGORITHMS];
	for (int i = 0; i < NR_OF_ALGORITHMS; i++)
	{
		goalNodes[i] = sf::CircleShape(0.4f*tileHeight);
		goalNodes[i].setPosition(sf::Vector2f(10.0f + goalPos[i]._x * (float)tileWidth, 10.0f + goalPos[i]._y * (float)tileHeight));
		goalNodes[i].setFillColor(sf::Color::Yellow);
	}

	//Other variables
	bool removePathFinding = false;
	bool calculatePaths = false;

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
		ImGui::ShowTestWindow();
		window.clear();

		/**************************************/
		/*          Start of GUI code         */
		/**************************************/

		if (ImGui::BeginMenu("Choose pathfinding"))
		{
			//TODO: Add interaction with the file system
			if (ImGui::BeginMenu("A*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &algorithmCombinations[0]);
				ImGui::MenuItem("Chebyshev", NULL, &algorithmCombinations[1]);
				ImGui::MenuItem("Octile", NULL, &algorithmCombinations[2]);
				ImGui::MenuItem("Euclidean", NULL, &algorithmCombinations[3]);
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Theta*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &algorithmCombinations[4]);
				ImGui::MenuItem("Chebyshev", NULL, &algorithmCombinations[5]);
				ImGui::MenuItem("Octile", NULL, &algorithmCombinations[6]);
				ImGui::MenuItem("Euclidean", NULL, &algorithmCombinations[7]);
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("HPA*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &algorithmCombinations[8]);
				ImGui::MenuItem("Chebyshev", NULL, &algorithmCombinations[9]);
				ImGui::MenuItem("Octile", NULL, &algorithmCombinations[10]);
				ImGui::MenuItem("Euclidean", NULL, &algorithmCombinations[11]);
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("IDA*", true))
			{
				ImGui::MenuItem("Manhattan", NULL, &algorithmCombinations[12]);
				ImGui::MenuItem("Chebyshev", NULL, &algorithmCombinations[13]);
				ImGui::MenuItem("Octile", NULL, &algorithmCombinations[14]);
				ImGui::MenuItem("Euclidean", NULL, &algorithmCombinations[15]);
				ImGui::EndMenu();
			}

			for (int i = 0; i < NR_OF_ALGORITHMS; i++)
			{
				for (int j = 0; j < NR_OF_HEURISTICS; j++)
				{
					if (algorithmCombinations[i*NR_OF_ALGORITHMS + j])
					{
						algorithmUsed[i] = true;
						heuristicUsed[j] = true;
					}
				}
			}

			ImGui::EndMenu();
		}
		if (calculatePaths && algorithmUsed[0])  //A*
		{
			if (heuristicUsed[0])
			{
				CalculateAStar(metrics, Pathfinding::MANHATTAN, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos[0], goalPos[0], grid, path, AStar_pathLength);
			}
			else if (heuristicUsed[1])
			{
				CalculateAStar(metrics, Pathfinding::CHEBYSHEV, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos[0], goalPos[0], grid, path, AStar_pathLength);
			}
			else if (heuristicUsed[2])
			{
				CalculateAStar(metrics, Pathfinding::OCTILE, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos[0], goalPos[0], grid, path, AStar_pathLength);
			}
			else if (heuristicUsed[3])
			{
				CalculateAStar(metrics, Pathfinding::EUCLIDEAN, AStar_pathTiles, AStar_openedTiles, AStar_expandedTiles, width, height, startPos[0], goalPos[0], grid, path, AStar_pathLength);
			}

			SaveDataToFile(metrics, algorithmUsed, heuristicUsed);
		}
		if (calculatePaths && algorithmUsed[1])  //Theta*
		{
			if (heuristicUsed[0])
			{
				CalculateThetaStar(metrics, Pathfinding::MANHATTAN, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
			else if (heuristicUsed[1])
			{
				CalculateThetaStar(metrics, Pathfinding::CHEBYSHEV, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
			else if (heuristicUsed[2])
			{
				CalculateThetaStar(metrics, Pathfinding::OCTILE, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}
			else if (heuristicUsed[3])
			{
				CalculateThetaStar(metrics, Pathfinding::EUCLIDEAN, ThetaStar_pathTiles, ThetaStar_openedTiles, ThetaStar_expandedTiles, width, height);
			}

			SaveDataToFile(metrics, algorithmUsed, heuristicUsed);
		}
		if (calculatePaths && algorithmUsed[2])  //HPA*
		{
			//if (heuristicUsed[0])
			//{
			//	CalculateHPAStar(metrics, Pathfinding::MANHATTAN, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//else if (heuristicUsed[1])
			//{
			//	CalculateHPAStar(metrics, Pathfinding::CHEBYSHEV, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//else if (heuristicUsed[2])
			//{
			//	CalculateHPAStar(metrics, Pathfinding::OCTILE, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//else if (heuristicUsed[3])
			//{
			//	CalculateHPAStar(metrics, Pathfinding::EUCLIDEAN, HPAabstractGraph, HPAopenedGraph, HPAexpandedGraph, width, height);
			//}
			//
			//SaveDataToFile(metrics, algorithmUsed, heuristicUsed);
		}
		if (calculatePaths && algorithmUsed[3])  //IDA*
		{
			if (heuristicUsed[0])
			{
				CalculateIDAStar(metrics, Pathfinding::MANHATTAN, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
			else if (heuristicUsed[1])
			{
				CalculateIDAStar(metrics, Pathfinding::CHEBYSHEV, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
			else if (heuristicUsed[2])
			{
				CalculateIDAStar(metrics, Pathfinding::OCTILE, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}
			else if (heuristicUsed[3])
			{
				CalculateIDAStar(metrics, Pathfinding::EUCLIDEAN, IDAStar_pathTiles, IDAStar_openedTiles, IDAStar_expandedTiles, width, height);
			}

			SaveDataToFile(metrics, algorithmUsed, heuristicUsed);
		}
		if (calculatePaths)
		{
			calculatePaths = false;
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
					//TODO ändra om så rätt algoritm ändrar sina start/goal nodes

					if (SetPosition == 0)
					{
						startPos[0] = pos;
						startNodes[0].setPosition(sf::Vector2f(10.0f + startPos[0]._x * (float)tileWidth, 10.0f + startPos[0]._y * (float)tileHeight));
					}
					if (SetPosition == 1)
					{
						startPos[1] = pos;
						startNodes[1].setPosition(sf::Vector2f(10.0f + startPos[1]._x * (float)tileWidth, 10.0f + startPos[1]._y * (float)tileHeight));
					}
					if (SetPosition == 2)
					{
						startPos[2] = pos;
						startNodes[2].setPosition(sf::Vector2f(10.0f + startPos[2]._x * (float)tileWidth, 10.0f + startPos[2]._y * (float)tileHeight));
					}
					if (SetPosition == 3)
					{
						startPos[3] = pos;
						startNodes[3].setPosition(sf::Vector2f(10.0f + startPos[3]._x * (float)tileWidth, 10.0f + startPos[3]._y * (float)tileHeight));
					}
				}
				else if (startOrGoal == 1)  //Goal pos
				{
					if (SetGoal == 0)
					{
						goalPos[0] = pos;
						goalNodes[0].setPosition(sf::Vector2f(10.0f + goalPos[0]._x * (float)tileWidth, 10.0f + goalPos[0]._y * (float)tileHeight));
					}
					if (SetGoal == 1)
					{
						goalPos[1] = pos;
						goalNodes[1].setPosition(sf::Vector2f(10.0f + goalPos[1]._x * (float)tileWidth, 10.0f + goalPos[1]._y * (float)tileHeight));
					}
					if (SetGoal == 2)
					{
						goalPos[2] = pos;
						goalNodes[2].setPosition(sf::Vector2f(10.0f + goalPos[2]._x * (float)tileWidth, 10.0f + goalPos[2]._y * (float)tileHeight));
					}
					if (SetGoal == 3)
					{
						goalPos[3] = pos;
						goalNodes[3].setPosition(sf::Vector2f(10.0f + goalPos[3]._x * (float)tileWidth, 10.0f + goalPos[3]._y * (float)tileHeight));
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
		if (ImGui::BeginMenu("Scale map"))
		{

			//TODO gör så den här fungerar


			ImGui::SliderInt("Change map scale", &tileWidth, 1, 10);
			ImGui::EndMenu();
		}
		ImGui::MenuItem("Calculate paths", NULL, &calculatePaths);
		
		if (removePathFinding)
		{
			for (int i = 0; i < NR_OF_ALGORITHMS; i++)
			{
				algorithmUsed[i] = false;
			}
			for (int j = 0; j < NR_OF_HEURISTICS; j++)
			{
				heuristicUsed[j] = false;
			}
			for (int i = 0; i < NR_OF_ALGORITHMS * NR_OF_HEURISTICS; i++)
			{
				algorithmCombinations[i] = false;
			}

			removePathFinding = false;
		}

		/**************************************/
		/*            End of GUI code         */
		/**************************************/

		//Draw the start and goal node(s)
		if (algorithmUsed[0])
		{
			window.draw(startNodes[0]);
			window.draw(goalNodes[0]);
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
		if (algorithmUsed[1])
		{
			window.draw(startNodes[1]);
			window.draw(goalNodes[1]);
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
		if (algorithmUsed[2])
		{
			window.draw(startNodes[2]);
			window.draw(goalNodes[2]);
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
		if (algorithmUsed[3])
		{
			window.draw(startNodes[3]);
			window.draw(goalNodes[3]);
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

void SaveDataToFile(Metrics metrics, bool algorithmUsed[NR_OF_ALGORITHMS], bool heuristicUsed[NR_OF_HEURISTICS])
{
	ofstream saveFile;
	saveFile.open("Metrics/metrics000.txt");

	//Which algorithm is used
	saveFile << "Algorithm used: ";
	if (algorithmUsed[0])
	{
		saveFile << "A Star. ";
	}
	if (algorithmUsed[1])
	{
		saveFile << "Theta Star. ";
	}
	if (algorithmUsed[2])
	{
		saveFile << "HPA Star. ";
	}
	if (algorithmUsed[3])
	{
		saveFile << "IDA Star. ";
	}

	//In combination with which heuristic is being used
	saveFile << "\nHeuristic used: ";
	if (heuristicUsed[0])
	{
		saveFile << "Manhattan. ";
	}
	if (heuristicUsed[1])
	{
		saveFile << "Chebyshev. ";
	}
	if (heuristicUsed[2])
	{
		saveFile << "Octile. ";
	}
	if (heuristicUsed[3])
	{
		saveFile << "Euclidean. ";
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

	//
}
void CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	/**/
}