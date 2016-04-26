#include <imgui.h>
#include <imgui-events-SFML.h>
#include <imgui-rendering-SFML.h>
#include <SFML/Graphics.hpp>
#include "AStar.h"
#include "ThetaStar.h"
#include "HPAStar.h"
#include "Metrics.h"
#include "MapReader.h"

string* GenerateMap(int width, int height, float obstacleDensity, MapReader &mr);
bool CalculateAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateThetaStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateHPAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateIDAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateGAAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);
bool CalculateDStarLite(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, int width, int height);

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;

	//Visual size of tiles
	int tileWidth = 2;
	int tileHeight = 2;

	//AI variables
	int pathLength = 0;
	int pathLength2 = 0;
	MapReader mr = MapReader();
	Vec2D startPos = { 1, 1 };
	Vec2D goalPos = { 479, 479 };
	Vec2D* path = nullptr;

	//Map data
	string* map = nullptr;
	map = mr.ReadMap("Maps/Randomized480x480-10-0.map");
	//map = GenerateMap(10, 10, 0.1f, mr);
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

	sf::Vertex* abstractGraph = nullptr;
	sf::Vertex* openedGraph = nullptr;
	sf::Vertex* expandedGraph = nullptr;
	
	/*expandedTiles = new sf::RectangleShape[ThetaStar_metrics.getNrOfExpandedNodes()];
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
		//	
		//	if (false)
		//	{
		//		sf::CircleShape ai(0.4f * tileHeight);
		//		ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
		//		ai.setFillColor(sf::Color::Red);

		//		CalculateAStar(metrics, AStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		//	}
		//	else if (false)
		//	{
		//		sf::CircleShape ai(0.4f * tileHeight);
		//		ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
		//		ai.setFillColor(sf::Color::Blue);

		//		CalculateThetaStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		//	}
		//	else if (false)
		//	{
		//		sf::CircleShape ai(0.4f * tileHeight);
		//		ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
		//		ai.setFillColor(sf::Color::Green);

		//		CalculateHPAStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		//	}
		//	else if (false)
		//	//{
		//	{
		//		sf::CircleShape ai(0.4f * tileHeight);
		//		ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
		//		ai.setFillColor(sf::Color::Magenta);

		//		CalculateIDAStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		//	}
		//	else if (false)
		//	{
		//		sf::CircleShape ai(0.4f * tileHeight);
		//		ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
		//		ai.setFillColor(sf::Color::Cyan);

		//		CalculateGAAStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		//	}
		//	else if (false)
		//	{
		//		sf::CircleShape ai(0.4f * tileHeight);
		//		ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
		//		ai.setFillColor(sf::Color::White);

		//		CalculateDStarLite(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		//	}


		window.clear();

		//window.draw(ai2);
		window.draw(goal);
		for (int i = 0; i < nrOfWalls; i++)  //Draw all the walls
		{
			window.draw(walls[i]);
		}
		//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)  //Draw Theta* opened nodes
		//{
		//	window.draw(openedTiles[i]);
		//}
		//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)  //Draw Theta* expanded nodes
		//{
		//	window.draw(expandedTiles[i]);
		//}
		//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)  //Draw A* opened nodes
		//{
		//	window.draw(openedTiles[i]);
		//}
		//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)  //Draw A* expanded nodes
		//{
		//	window.draw(expandedTiles[i]);
		//}

		//window.draw(pathTiles2, pathLength2 + 1, sf::LinesStrip);
		//window.draw(pathTiles, pathLength + 1, sf::LinesStrip);

		//window.draw(ai);
		ImGui::Render();
		window.display();
	}
	//pathFinding.cleanMap();
	delete[] expandedGraph;
	delete[] openedGraph;
	delete[] abstractGraph;
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
//bool CalculateAStar(Metrics metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
//{
//	//TODO loopa över hela mapen och sätt traversable
//	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
//
//	AStar pathFinding(width, height, { 0, 0 }, startPos, goalPos, grid, heuristic);
//	pathFinding.init(startPos, goalPos);
//
//	initiateMap(width, height);
//
//	if (pathFinding.findPath(metrics))
//	{
//		pathLength = pathFinding.getNrOfPathNodes();
//		path = pathFinding.getPath();
//	}
//	openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
//	for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
//	{
//		openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
//		openedTiles[i].setFillColor(sf::Color(0, 200, 200, 120));
//		openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
//	}
//	expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
//	for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
//	{
//		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
//		expandedTiles[i].setFillColor(sf::Color(200, 0, 0, 120));
//		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
//	}
//	pathTiles = new sf::Vertex[pathLength + 1];
//	for (int i = 0; i < pathLength; i++)
//	{
//		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
//		pathTiles[i].color = sf::Color(200, 0, 200, 255);
//	}
//	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
//	
//	return false;
//}
//bool CalculateThetaStar(Metrics metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
//{
//	//TODO loopa över hela mapen och sätt traversable
//	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
//
//	ThetaStar pathFinding2(width, height, startPos, goalPos, heuristic);
//	pathFinding2.init(startPos, goalPos);
//
//	AStar pathFinding(width, height, startPos, goalPos, heuristic);
//	initiateMap(width, height);
//
//	if (pathFinding.findPath(metrics))
//	{
//		pathLength = pathFinding.getNrOfPathNodes();
//		path = pathFinding.getPath();
//	}
//	openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
//	for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
//	{
//		openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
//		openedTiles[i].setFillColor(sf::Color(0, 0, 200, 120));
//		openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
//	}
//	expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
//	for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
//	{
//		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
//		expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
//		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
//	}
//	pathTiles = new sf::Vertex[pathLength + 1];
//	for (int i = 0; i < pathLength; i++)
//	{
//		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
//		pathTiles[i].color = sf::Color(200, 200, 0, 255);
//	}
//	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
//	return false;
//}
//bool CalculateHPAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
//{
//	//TODO loopa över hela mapen och sätt traversable
//	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
//	HPAStar pathFinding3(width, height, clusterSize, Pathfinding::OCTILE);
//	pathFinding3.init(startPos, goalPos);
//
//	initiateMap(width, height);
//
//	/**/
//	return false;
//}
//bool CalculateIDAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
//{
//	//TODO loopa över hela mapen och sätt traversable
//	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
//
//	initiateMap(width, height);
//
//	/**/
//	return false;
//}
//bool CalculateGAAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
//{
//	//TODO loopa över hela mapen och sätt traversable
//	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
//
//	initiateMap(width, height);
//
//	/**/
//	return false;
//}
//bool CalculateDStarLite(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map, int width, int height)
//{
//	//TODO loopa över hela mapen och sätt traversable
//	//pathFinding.setTraversable({ (i % width), (i / width) }, true);
//
//	initiateMap(width, height);
//
//	/**/
//	return false;
//}