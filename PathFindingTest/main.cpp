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

string* GenerateMap(int width, int height, float obstacleDensity, MapReader &mr);
bool CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height);
bool CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height);
bool CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height);
bool CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height);

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;

	//Visual size of tiles
	int tileWidth = 10;
	int tileHeight = 10;

	//AI variables
	int pathLength = 0;
	int pathLength2 = 0;
	MapReader mr = MapReader();
	Vec2D startPos =  { 0,0 };
	Vec2D startPos2 = { 0,2 };
	Vec2D startPos3 = { 2,0 };
	Vec2D startPos4 = { 2,2 };
	Vec2D goalPos = { 479, 479 };
	Vec2D* path = nullptr;

	//Map data
	string* map = nullptr;
	map = mr.ReadMap("Maps/Randomized10x10-0-0.map");
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

	sf::CircleShape goal(0.4f * tileHeight);
	goal.setFillColor(sf::Color::Yellow);
	goal.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)tileWidth, 10.0f + goalPos._y * (float)tileHeight));

	sf::RectangleShape* openedTiles = nullptr;
	sf::RectangleShape* expandedTiles = nullptr;
	sf::Vertex* pathTiles = nullptr;

	/////////////////////////////////////////////////////////////////////////////

	sf::RectangleShape* openedTiles2 = nullptr;
	sf::RectangleShape* expandedTiles2 = nullptr;
	sf::Vertex* pathTiles2 = nullptr;

	//HPA*
	sf::Vertex* HPAabstractGraph = nullptr;
	sf::Vertex* HPAopenedGraph = nullptr;
	sf::Vertex* HPAexpandedGraph = nullptr;

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
	bool randomizeMap = false;
	static char widthBuffer[4] = "512";
	static char heightBuffer[4] = "512";
	static char densityBuffer[3] = "30";

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
			//

			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Yet another option"))
		{
			//
			ImGui::EndMenu();
		}

		ImGui::ShowTestWindow();

		if (AStarManhattan || AStarChebyshev || AStarOctile || AStarEuclidean)  //A*
		{
			AStar_start.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			AStar_start.setFillColor(sf::Color::Red);

			if (AStarManhattan)
			{
				CalculateAStar(AStar_metrics, Pathfinding::MANHATTAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (AStarChebyshev)
			{
				CalculateAStar(AStar_metrics, Pathfinding::CHEBYSHEV, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (AStarOctile)
			{
				CalculateAStar(AStar_metrics, Pathfinding::OCTILE, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (AStarEuclidean)
			{
				CalculateAStar(AStar_metrics, Pathfinding::EUCLIDEAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}

			window.draw(AStar_start);
		}
		if (ThetaStarManhattan || ThetaStarChebyshev ||ThetaStarOctile || ThetaStarEuclidean)  //Theta*
		{
			ThetaStar_start.setPosition(sf::Vector2f(10.0f + startPos2._x * (float)tileWidth, 10.0f + startPos2._y * (float)tileHeight));
			ThetaStar_start.setFillColor(sf::Color::Blue);

			if (ThetaStarManhattan)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::MANHATTAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (ThetaStarChebyshev)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::CHEBYSHEV, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (ThetaStarOctile)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::OCTILE, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (ThetaStarEuclidean)
			{
				CalculateThetaStar(ThetaStar_metrics, Pathfinding::EUCLIDEAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}

			window.draw(ThetaStar_start);
		}
		if (HPAStarManhattan || HPAStarChebyshev || HPAStarOctile || HPAStarEuclidean)  //HPA*
		{
			HPAStar_start.setPosition(sf::Vector2f(10.0f + startPos3._x * (float)tileWidth, 10.0f + startPos3._y * (float)tileHeight));
			HPAStar_start.setFillColor(sf::Color::Green);

			if (HPAStarManhattan)
			{
				CalculateHPAStar(HPAStar_metrics, Pathfinding::MANHATTAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (HPAStarChebyshev)
			{
				CalculateHPAStar(HPAStar_metrics, Pathfinding::CHEBYSHEV, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (HPAStarOctile)
			{
				CalculateHPAStar(HPAStar_metrics, Pathfinding::OCTILE, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (HPAStarEuclidean)
			{
				CalculateHPAStar(HPAStar_metrics, Pathfinding::EUCLIDEAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}

			window.draw(HPAStar_start);
		}
		if (IDAStarManhattan || IDAStarChebyshev || IDAStarOctile || IDAStarEuclidean)  //IDA*
		{
			IDAStar_start.setPosition(sf::Vector2f(10.0f + startPos4._x * (float)tileWidth, 10.0f + startPos4._y * (float)tileHeight));
			IDAStar_start.setFillColor(sf::Color::Magenta);

			if (IDAStarManhattan)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::MANHATTAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (IDAStarChebyshev)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::CHEBYSHEV, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (IDAStarOctile)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::OCTILE, pathTiles, openedTiles, expandedTiles, map, width, height);
			}
			else if (IDAStarEuclidean)
			{
				CalculateIDAStar(IDAStar_metrics, Pathfinding::EUCLIDEAN, pathTiles, openedTiles, expandedTiles, map, width, height);
			}

			window.draw(IDAStar_start);
		}

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
		}

		/**************************************/
		/*            End of GUI code         */
		/**************************************/

		//Draw the goal node(s)
		window.draw(goal);

		//Draw all the walls
		for (int i = 0; i < nrOfWalls; i++)
		{
			window.draw(walls[i]);
		}
		//Draw Theta* opened nodes
		//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
		//{
		//	window.draw(openedTiles[i]);
		//}
		//Draw Theta* expanded nodes
		//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
		//{
		//	window.draw(expandedTiles[i]);
		//}
		//Draw A* opened nodes
		//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
		//{
		//	window.draw(openedTiles[i]);
		//}
		//Draw A* expanded nodes
		//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
		//{
		//	window.draw(expandedTiles[i]);
		//}

		//Draw the lines that the pathfinding are traversing
		//window.draw(pathTiles2, pathLength2 + 1, sf::LinesStrip);
		//window.draw(pathTiles, pathLength + 1, sf::LinesStrip);

		ImGui::Render();
		window.display();
	}

	//pathFinding.cleanMap();
	delete[] HPAexpandedGraph;
	delete[] HPAopenedGraph;
	delete[] HPAabstractGraph;
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
bool CalculateAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	//AStar pathFinding(width, height, { 0, 0 }, startPos, goalPos, grid, heuristic);
	//pathFinding.init(startPos, goalPos);

	//if (pathFinding.findPath(metrics))
	//{
	//	pathLength = pathFinding.getNrOfPathNodes();
	//	path = pathFinding.getPath();
	//}
	//openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
	//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	//{
	//	openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
	//	openedTiles[i].setFillColor(sf::Color(0, 200, 200, 120));
	//	openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
	//}
	//expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
	//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	//{
	//	expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
	//	expandedTiles[i].setFillColor(sf::Color(200, 0, 0, 120));
	//	expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
	//}
	//pathTiles = new sf::Vertex[pathLength + 1];
	//for (int i = 0; i < pathLength; i++)
	//{
	//	pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
	//	pathTiles[i].color = sf::Color(200, 0, 200, 255);
	//}
	//pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
	//
	return false;
}
bool CalculateThetaStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
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
bool CalculateHPAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
	//HPAStar pathFinding3(width, height, clusterSize, Pathfinding::OCTILE);
	//pathFinding3.init(startPos, goalPos);

	//
	return false;
}
bool CalculateIDAStar(Metrics &metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
{
	//TODO loopa över hela mapen och sätt traversable
	//pathFinding.setTraversable({ (i % width), (i / width) }, true);

	/**/
	return false;
}