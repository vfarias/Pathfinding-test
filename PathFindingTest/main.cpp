#include <imgui.h>
#include <imgui-events-SFML.h>
#include <imgui-rendering-SFML.h>
#include <SFML/Graphics.hpp>
#include "AStar.h"
#include "ThetaStar.h"
#include "Metrics.h"
#include "MapReader.h"

//Visual size of tiles
int tileWidth = 2;
int tileHeight = 2;

//Map data
int width = 0;
int height = 0;
int nrOfWalls = 0;
int pathLength = 0;
Vec2D* path = nullptr;
Vec2D* wallPos = new Vec2D[nrOfWalls];
sf::RectangleShape* walls = new sf::RectangleShape[nrOfWalls];

//Map positions
Vec2D startPos = {1, 1};
Vec2D goalPos = {479, 479};

bool initiateMap();
bool CalculateAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles);
bool CalculateThetaStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles);
bool CalculateHPAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles);
bool CalculateIDAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles);
bool CalculateGAAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles);
bool CalculateDStarLite(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles);

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;

	//AI variables
	int pathLength = 0;
	int pathLength2 = 0;
	MapReader mr = MapReader();

	string* map = nullptr;
	//int width = 480;
	//int height = 480;
	//float obstacleDensity = 1.00f;
	//mr.GenerateRandomMap(width, height, obstacleDensity);
	//map = mr.ReadMap("Maps/Randomized480x480-100-0.map");

	map = mr.ReadMap("Maps/maze512-1-5.map");
	width = mr.GetWidth();
	height = mr.GetHeight();
	nrOfWalls = mr.GetNrOfWalls(map);

	Metrics metrics = Metrics();

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
	
	
	//First pathfinding algorithm
	if (pathFinding.findPath(metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}
	openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		openedTiles[i].setFillColor(sf::Color(0,0,200, 120));
		openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
	}
	expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
	}
	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
	
	
	
	//Second pathfinding algorithm
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
		
		if (false)
		{
			sf::CircleShape ai(0.4f * tileHeight);
			ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			ai.setFillColor(sf::Color::Red);

			CalculateAStar(metrics, AStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		}
		else if (false)
		{
			sf::CircleShape ai(0.4f * tileHeight);
			ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			ai.setFillColor(sf::Color::Blue);

			CalculateThetaStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		}
		else if (false)
		{
			sf::CircleShape ai(0.4f * tileHeight);
			ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			ai.setFillColor(sf::Color::Green);

			CalculateHPAStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		}
		else if (false)
		{
			sf::CircleShape ai(0.4f * tileHeight);
			ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			ai.setFillColor(sf::Color::Magenta);

			CalculateIDAStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		}
		else if (false)
		{
			sf::CircleShape ai(0.4f * tileHeight);
			ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			ai.setFillColor(sf::Color::Cyan);

			CalculateGAAStar(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		}
		else if (false)
		{
			sf::CircleShape ai(0.4f * tileHeight);
			ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
			ai.setFillColor(sf::Color::White);

			CalculateDStarLite(metrics, ThetaStar::MANHATTAN, pathTiles, openedTiles, expandedTiles);
		}


		window.clear();
		
		//window.draw(ai2);
		window.draw(goal);
		//for (int i = 0; i < nrOfWalls; i++)  //Draw all the walls
		//{
		//	window.draw(walls[i]);
		//}
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

		window.draw(pathTiles, pathLength + 1, sf::LinesStrip);

		//window.draw(ai);
		ImGui::Render();
		window.display();
	}
	//pathFinding.cleanMap();
	delete[] expandedTiles;
	delete[] openedTiles;
	delete[] pathTiles;
	delete[] walls;
	ImGui::SFML::Shutdown();
	return 0;
}

bool initiateMap()
{
	int counter = 0;

	for (int i = 0; i < width*height; i++)
	{
		pathFinding.setTraversable({ (i % width), (i / width) }, true);

		if (map != nullptr)
		{
			if (map[i] == "@")
			{
				wallPos[counter++] = { (i % width), (i / width) };
			}
		}
	}

	for (int i = 0; i < nrOfWalls; i++)
	{
		walls[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		walls[i].setFillColor(sf::Color::White);
		walls[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * wallPos[i]._x, 10.0f + (float)tileHeight * wallPos[i]._y));
		pathFinding.setTraversable(wallPos[i], false);
	}

	return false;
}
bool CalculateAStar(Metrics metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map)
{
	ThetaStar pathFinding(width, height, startPos, goalPos, heuristic);
	initiateMap();

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
bool CalculateThetaStar(Metrics metrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map)
{
	AStar pathFinding(width, height, startPos, goalPos, heuristic);
	initiateMap();

	if (pathFinding.findPath(metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}
	openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		openedTiles[i].setFillColor(sf::Color(0, 0, 200, 120));
		openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getOpenedNodes()[i]._y));
	}
	expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * metrics.getExpandedNodes()[i]._y));
	}
	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
	return false;
}
bool CalculateHPAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map)
{

	initiateMap();

	/**/
	return false;
}
bool CalculateIDAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map)
{

	initiateMap();

	/**/
	return false;
}
bool CalculateGAAStar(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map)
{

	initiateMap();

	/**/
	return false;
}
bool CalculateDStarLite(Metrics AStarMetrics, Pathfinding::Heuristic heuristic, sf::Vertex* pathTiles, sf::RectangleShape* openedTiles, sf::RectangleShape* expandedTiles, string* map)
{

	initiateMap();

	/**/
	return false;
}