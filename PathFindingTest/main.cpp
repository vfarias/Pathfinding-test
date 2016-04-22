#include <imgui.h>
#include <imgui-events-SFML.h>
#include <imgui-rendering-SFML.h>
#include <SFML/Graphics.hpp>
#include "AStar.h"
#include "ThetaStar.h"
#include "Metrics.h"
#include "MapReader.h"

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;
	//Visual size of tiles
	int tileWidth = 2;
	int tileHeight = 2;

	//AI variables
	Vec2D startPos = {1, 1};
	//Vec2D startPos2 = {511, 1};
	//Vec2D goalPos = {511, 511};
	//Vec2D goalPos2 = {511, 511};
	Vec2D startPos2 = {1, 1};
	Vec2D goalPos = {479, 479};
	Vec2D goalPos2 = {479, 479};
	int pathLength = 0;
	int pathLength2 = 0;
	Vec2D* path = nullptr;
	Vec2D* path2 = nullptr;
	MapReader mr = MapReader();
	string* map = nullptr;

	//int width = 480;
	//int height = 480;
	//float obstacleDensity = 1.00f;
	//mr.GenerateRandomMap(width, height, obstacleDensity);
	//map = mr.ReadMap("Maps/Randomized480x480-100-0.map");

	map = mr.ReadMap("Maps/maze512-1-5.map");
	int width = mr.GetWidth();
	int height = mr.GetHeight();
	int nrOfWalls = mr.GetNrOfWalls(map);
	Vec2D* WallPos = new Vec2D[nrOfWalls];
	int counter = 0;

	ThetaStar pathFinding(width, height, startPos, goalPos, ThetaStar::EUCLIDEAN);
	AStar pathFinding2(width, height, startPos, goalPos, AStar::OCTILE);

	/*for (int i = 0; i < HEIGHT; i++)
	{
		for (int j = 0; j < WIDTH; j++)
		{
			cout << map[i*WIDTH + j];
		}
		cout << endl;
	}*/

	//sf::RectangleShape* walls = new sf::RectangleShape[nrOfWalls];

	for (int i = 0; i < width*height; i++)
	{
		pathFinding.setTraversable({ (i % width), (i / width) }, true);
		pathFinding2.setTraversable({ (i % width), (i / width) }, true);

		if (map != nullptr)
		{
			if (map[i] == "@")
			{
				WallPos[counter++] = { (i % width), (i / width) };
			}
		}
	}

	for (int i = 0; i < nrOfWalls; i++)
	{
		//walls[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		//walls[i].setFillColor(sf::Color::White);
		//walls[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * WallPos[i]._x, 10.0f + (float)tileHeight * WallPos[i]._y));
		pathFinding.setTraversable(WallPos[i], false);
		pathFinding2.setTraversable(WallPos[i], false);
	}

	Metrics ThetaStar_metrics = Metrics();
	Metrics AStar_metrics = Metrics();

	sf::RenderWindow window(sf::VideoMode(800, 600), "AI test");
	window.setFramerateLimit(60);

	ImGui::SFML::SetRenderTarget(window);
	ImGui::SFML::InitImGuiRendering();
	ImGui::SFML::SetWindow(window);
	ImGui::SFML::InitImGuiEvents();

	//sf::CircleShape shape(100.f);
	//

	//sf::RectangleShape grid[GRID_WIDTH][GRID_HEIGHT];

	sf::CircleShape ai(0.4f * tileHeight);
	ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)tileWidth, 10.0f + startPos._y * (float)tileHeight));
	ai.setFillColor(sf::Color::Red);

	sf::CircleShape goal(0.4f * tileHeight);
	//goal.setOutlineThickness(2.0);
	goal.setFillColor(sf::Color::Blue);
	goal.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)tileWidth, 10.0f + goalPos._y * (float)tileHeight));
	
	//2nd ai
	sf::CircleShape ai2(0.4f * tileHeight);
	ai2.setPosition(sf::Vector2f(10.0f + startPos2._x * (float)tileWidth, 10.0f + startPos2._y * (float)tileHeight));
	ai2.setFillColor(sf::Color::Green);

	sf::RectangleShape* openedTiles = nullptr;
	sf::RectangleShape* expandedTiles = nullptr;
	sf::RectangleShape* openedTiles2 = nullptr;
	sf::RectangleShape* expandedTiles2 = nullptr;
	sf::Vertex* pathTiles = nullptr;
	sf::Vertex* pathTiles2 = nullptr;
	
	/*
	//First pathfinding algorithm
	if (pathFinding.findPath(ThetaStar_metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}
	openedTiles = new sf::RectangleShape[ThetaStar_metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < ThetaStar_metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		openedTiles[i].setFillColor(sf::Color(0,0,200, 120));
		openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * ThetaStar_metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * ThetaStar_metrics.getOpenedNodes()[i]._y));
	}
	expandedTiles = new sf::RectangleShape[ThetaStar_metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < ThetaStar_metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
		expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * ThetaStar_metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * ThetaStar_metrics.getExpandedNodes()[i]._y));
	}
	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path[i]._x + 0.5f), 10.0f + (float)tileHeight * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)tileWidth * (startPos._x + 0.5f), 10.0f + (float)tileHeight * (startPos._y + 0.5f));
	*/
	
	/*
	//Second pathfinding algorithm
	if (pathFinding2.findPath(AStar_metrics))
	{
		pathLength2 = pathFinding2.getNrOfPathNodes();
		path2 = pathFinding2.getPath();
	}
	openedTiles2 = new sf::RectangleShape[AStar_metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < AStar_metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles2[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		openedTiles2[i].setFillColor(sf::Color(0, 200, 200, 120));
		openedTiles2[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * AStar_metrics.getOpenedNodes()[i]._x, 10.0f + (float)tileHeight * AStar_metrics.getOpenedNodes()[i]._y));
	}
	expandedTiles2 = new sf::RectangleShape[AStar_metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < AStar_metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles2[i] = sf::RectangleShape(sf::Vector2f((float)tileWidth, (float)tileHeight));
		expandedTiles2[i].setFillColor(sf::Color(200, 0, 0, 120));
		expandedTiles2[i].setPosition(sf::Vector2f(10.0f + (float)tileWidth * AStar_metrics.getExpandedNodes()[i]._x, 10.0f + (float)tileHeight * AStar_metrics.getExpandedNodes()[i]._y));
	}
	pathTiles2 = new sf::Vertex[pathLength2 + 1];
	for (int i = 0; i < pathLength2; i++)
	{
		pathTiles2[i] = sf::Vertex(sf::Vector2f(10.0f + (float)tileWidth * (path2[i]._x + 0.5f), 10.0f + (float)tileHeight * (path2[i]._y + 0.5f)));
		pathTiles2[i].color = sf::Color(200, 0, 200, 255);
	}
	pathTiles2[pathLength2] = sf::Vector2f(10.0f + (float)tileWidth * (startPos2._x + 0.5f), 10.0f + (float)tileHeight * (startPos2._y + 0.5f));
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
		window.clear();
		
		//window.draw(ai2);
		window.draw(goal);
		//for (int i = 0; i < nrOfWalls; i++)  //Draw all the walls
		//{
		//	window.draw(walls[i]);
		//}
		//for (int i = 0; i < ThetaStar_metrics.getNrOfOpenedNodes(); i++)  //Draw Theta* opened nodes
		//{
		//	window.draw(openedTiles[i]);
		//}
		//for (int i = 0; i < ThetaStar_metrics.getNrOfExpandedNodes(); i++)  //Draw Theta* expanded nodes
		//{
		//	window.draw(expandedTiles[i]);
		//}
		//for (int i = 0; i < AStar_metrics.getNrOfOpenedNodes(); i++)  //Draw A* opened nodes
		//{
		//	window.draw(openedTiles2[i]);
		//}
		//for (int i = 0; i < AStar_metrics.getNrOfExpandedNodes(); i++)  //Draw A* expanded nodes
		//{
		//	window.draw(expandedTiles2[i]);
		//}

		window.draw(pathTiles, pathLength + 1, sf::LinesStrip);
		window.draw(pathTiles2, pathLength2 + 1, sf::LinesStrip);

	//	window.draw(ai);
		ImGui::Render();
		window.display();
	}
	//pathFinding.cleanMap();
	delete[] expandedTiles;
	delete[] openedTiles;
	delete[] expandedTiles2;
	delete[] openedTiles2;
	delete[] pathTiles;
	//delete[] walls;
	delete[] map;
	delete[] WallPos;
	ImGui::SFML::Shutdown();
	return 0;
}