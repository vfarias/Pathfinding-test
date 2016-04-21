#include <imgui.h>
#include <imgui-events-SFML.h>
#include <imgui-rendering-SFML.h>
#include <SFML/Graphics.hpp>
#include "AStar.h"
#include "ThetaStar.h"
#include "HPAStar.h"
#include "Metrics.h"
#include "MapReader.h"

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;
	//Visual size of tiles
	const int TILE_WIDTH = 2;
	const int TILE_HEIGHT = 2;
	Vec2D startPos = {1, 1};
	Vec2D goalPos = {99, 99};
	Vec2D startPos2 = {1, 10};
	int pathLength = 0;
	Vec2D* path = nullptr;

	/**************************/
	MapReader mr = MapReader();
	const int WIDTH = 100;
	const int HEIGHT = 100;
	const float OBSTACLE_DENSITY = 1.00f;
	//mr.GenerateRandomMap(WIDTH, HEIGHT, OBSTACLE_DENSITY);

	const int nrOfWalls = (int)(WIDTH * HEIGHT * OBSTACLE_DENSITY);
	string* map = nullptr;
	//map = mr.ReadMap("Maps/Randomized20x20-30-0.txt");
	map = mr.ReadMap("Maps/adaptive-depth-1.map");
	Vec2D* WallPos = new Vec2D[nrOfWalls];
	int counter = 0;
	int test = sizeof(sf::RectangleShape);

	HPAStar pathFinding(WIDTH, HEIGHT, 10, Pathfinding::OCTILE);
	pathFinding.init(startPos, goalPos);
	//ThetaStar pathFinding(WIDTH, HEIGHT, startPos, goalPos, ThetaStar::EUCLIDEAN);
	AStar pathFinding2(WIDTH, HEIGHT, startPos2, goalPos, AStar::OCTILE);

	/*for (int i = 0; i < HEIGHT; i++)
	{
		for (int j = 0; j < WIDTH; j++)
		{
			cout << map[i*WIDTH + j];
		}
		cout << endl;
	}*/

	sf::RectangleShape* walls = new sf::RectangleShape[nrOfWalls];

	for (int i = 0; i < WIDTH*HEIGHT; i++)
	{
		pathFinding.setTraversable({ (i % WIDTH), (i / WIDTH) }, true);

		if (map[i] == "@")
		{
			WallPos[counter++] = { (i % WIDTH), (i / WIDTH)};
		}
	}

	for (int i = 0; i < nrOfWalls; i++)
	{
		walls[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
		walls[i].setFillColor(sf::Color::White);
		walls[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * WallPos[i]._x, 10.0f + (float)TILE_HEIGHT * WallPos[i]._y));
		pathFinding.setTraversable(WallPos[i], false);
		pathFinding2.setTraversable(WallPos[i], false);
	}

	/**************************/

	Metrics metrics = Metrics();
	
	sf::RenderWindow window(sf::VideoMode(1300, 700), "AI test");
	window.setFramerateLimit(60);

	ImGui::SFML::SetRenderTarget(window);
	ImGui::SFML::InitImGuiRendering();
	ImGui::SFML::SetWindow(window);
	ImGui::SFML::InitImGuiEvents();

	//sf::CircleShape shape(100.f);
	//

	//sf::RectangleShape grid[GRID_WIDTH][GRID_HEIGHT];

	//sf::CircleShape ai(0.4f * TILE_HEIGHT);
	//ai.setPosition(sf::Vector2f(10.0f + startPos._x * (float)TILE_WIDTH, 10.0f + startPos._y * (float)TILE_HEIGHT));
	//ai.setFillColor(sf::Color::Red);

	sf::CircleShape goal(0.4f * TILE_HEIGHT);
	//goal.setOutlineThickness(2.0);
	goal.setFillColor(sf::Color::Blue);
	goal.setPosition(sf::Vector2f(10.0f + goalPos._x * (float)TILE_WIDTH, 10.0f + goalPos._y * (float)TILE_HEIGHT));
	
	//2nd ai
	//sf::CircleShape ai2(0.4f * TILE_HEIGHT);
	//ai2.setPosition(sf::Vector2f(10.0f + startPos2._x * (float)TILE_WIDTH, 10.0f + startPos2._y * (float)TILE_HEIGHT));
	//ai2.setFillColor(sf::Color::Green);

	sf::RectangleShape* openedTiles = nullptr;
	sf::RectangleShape* expandedTiles = nullptr;
	sf::Vertex* pathTiles = nullptr;
	if (pathFinding.findPath(metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}
	//openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
	//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	//{
	//	openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
	//	openedTiles[i].setFillColor(sf::Color(0,0,200, 120));
	//	openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * metrics.getOpenedNodes()[i]._x, 10.0f + (float)TILE_HEIGHT * metrics.getOpenedNodes()[i]._y));
	//}

	//
	//expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
	//for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	//{
	//	expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
	//	expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
	//	expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * metrics.getExpandedNodes()[i]._x, 10.0f + (float)TILE_HEIGHT * metrics.getExpandedNodes()[i]._y));
	//}

	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)TILE_WIDTH * (path[i]._x + 0.5f), 10.0f + (float)TILE_HEIGHT * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)TILE_WIDTH * (startPos._x + 0.5f), 10.0f + (float)TILE_HEIGHT * (startPos._y + 0.5f));
	//pathFinding2.findPath();
	//int pathLength2 = pathFinding2.getPathLength();
	//Vec2D* path2 = pathFinding2.getPath();

	while (window.isOpen())
	{
		ImGui::SFML::UpdateImGui();
		ImGui::SFML::UpdateImGuiRendering();
		sf::Event event;
		while (window.pollEvent(event))
		{
			ImGui::SFML::ProcessEvent(event);
			if (event.type == sf::Event::Closed)
				window.close();
		}
		ImGuiIO &io = ImGui::GetIO();
		
		ImGui::ShowTestWindow();

		window.clear();
		
		//window.draw(ai2);
		window.draw(goal);
		//for (int i = 0; i < nrOfWalls; i++)
		//{
		//	window.draw(walls[i]);
		//}
		//for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
		//{
		//	window.draw(openedTiles[i]);
		//}
		//for (int i = 0; i <  metrics.getNrOfExpandedNodes(); i++)
		//{
		//	window.draw(expandedTiles[i]);
		//}

		window.draw(pathTiles, pathLength + 1, sf::LinesStrip);

	//	window.draw(ai);
		ImGui::Render();
		window.display();
	}
	//pathFinding.cleanMap();
	delete[] expandedTiles;
	delete[] openedTiles;
	delete[] pathTiles;
	delete[] map;
	delete[] walls;
	delete[] WallPos;
	ImGui::SFML::Shutdown();
	return 0;
}