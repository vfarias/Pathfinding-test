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
	Vec2D goalPos = {511, 511};
	Vec2D startPos2 = {1, 1};
	int pathLength = 0;
	int pathLength2 = 0;
	Vec2D* path = nullptr;
	Vec2D* path2 = nullptr;
	MapReader mr = MapReader();
	//float obstacleDensity = 0.30f;
	//mr.GenerateRandomMap(width, height, obstacleDensity);

	string* map = nullptr;
	//map = mr.ReadMap("Maps/Randomized20x20-30-0.txt");
	map = mr.ReadMap("Maps/maze512-1-5.map");
	int width = mr.GetWidth();
	int height = mr.GetHeight();
	float obstacleDensity = 1.00f;
	int nrOfWalls = (int)(width * height * obstacleDensity);
	Vec2D* WallPos = new Vec2D[nrOfWalls];
	int counter = 0;
	int test = sizeof(sf::RectangleShape);

	HPAStar pathFinding(width, height, 16, Pathfinding::OCTILE);
	pathFinding.init(startPos, goalPos);
	//ThetaStar pathFinding(WIDTH, HEIGHT, startPos, goalPos, ThetaStar::EUCLIDEAN);
	AStar pathFinding2(width, height, startPos2, goalPos, AStar::OCTILE);

	/*for (int i = 0; i < HEIGHT; i++)
	{
	for (int j = 0; j < WIDTH; j++)
	{
	cout << map[i*WIDTH + j];
	}
	cout << endl;
	}*/

	sf::RectangleShape* walls = new sf::RectangleShape[nrOfWalls];

	for (int i = 0; i < width*height; i++)
	{
		pathFinding.setTraversable({(i % width), (i / width)}, true);
		pathFinding2.setTraversable({(i % width), (i / width)}, true);

		if (map[i] == "@")
		{
			WallPos[counter++] = {(i % width), (i / width)};
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

	Metrics ThetaStar_metrics = Metrics();
	Metrics AStar_metrics = Metrics();

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
	sf::RectangleShape* openedTiles2 = nullptr;
	sf::RectangleShape* expandedTiles2 = nullptr;
	sf::Vertex* pathTiles = nullptr;
	sf::Vertex* pathTiles2 = nullptr;
	if (pathFinding.findPath(ThetaStar_metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}
	if (pathFinding2.findPath(AStar_metrics))
	{
		pathLength2 = pathFinding2.getNrOfPathNodes();
		path2 = pathFinding2.getPath();
	}
	//openedTiles = new sf::RectangleShape[ThetaStar_metrics.getNrOfOpenedNodes()];
	//for (int i = 0; i < ThetaStar_metrics.getNrOfOpenedNodes(); i++)
	//{
	//	openedTiles[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
	//	openedTiles[i].setFillColor(sf::Color(0, 0, 200, 120));
	//	openedTiles[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * ThetaStar_metrics.getOpenedNodes()[i]._x, 10.0f + (float)TILE_HEIGHT * ThetaStar_metrics.getOpenedNodes()[i]._y));
	//}
	//expandedTiles = new sf::RectangleShape[ThetaStar_metrics.getNrOfExpandedNodes()];
	//for (int i = 0; i < ThetaStar_metrics.getNrOfExpandedNodes(); i++)
	//{
	//	expandedTiles[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
	//	expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
	//	expandedTiles[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * ThetaStar_metrics.getExpandedNodes()[i]._x, 10.0f + (float)TILE_HEIGHT * ThetaStar_metrics.getExpandedNodes()[i]._y));
	//}

	//openedTiles2 = new sf::RectangleShape[AStar_metrics.getNrOfOpenedNodes()];
	//for (int i = 0; i < AStar_metrics.getNrOfOpenedNodes(); i++)
	//{
	//	openedTiles2[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
	//	openedTiles2[i].setFillColor(sf::Color(0, 0, 200, 120));
	//	openedTiles2[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * AStar_metrics.getOpenedNodes()[i]._x, 10.0f + (float)TILE_HEIGHT * AStar_metrics.getOpenedNodes()[i]._y));
	//}
	//expandedTiles2 = new sf::RectangleShape[AStar_metrics.getNrOfExpandedNodes()];
	//for (int i = 0; i < AStar_metrics.getNrOfExpandedNodes(); i++)
	//{
	//	expandedTiles2[i] = sf::RectangleShape(sf::Vector2f((float)TILE_WIDTH, (float)TILE_HEIGHT));
	//	expandedTiles2[i].setFillColor(sf::Color(0, 200, 0, 120));
	//	expandedTiles2[i].setPosition(sf::Vector2f(10.0f + (float)TILE_WIDTH * AStar_metrics.getExpandedNodes()[i]._x, 10.0f + (float)TILE_HEIGHT * AStar_metrics.getExpandedNodes()[i]._y));
	//}

	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + (float)TILE_WIDTH * (path[i]._x + 0.5f), 10.0f + (float)TILE_HEIGHT * (path[i]._y + 0.5f)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles2 = new sf::Vertex[pathLength2 + 1];
	for (int i = 0; i < pathLength2; i++)
	{
		pathTiles2[i] = sf::Vertex(sf::Vector2f(10.0f + (float)TILE_WIDTH * (path2[i]._x + 0.5f), 10.0f + (float)TILE_HEIGHT * (path2[i]._y + 0.5f)));
		pathTiles2[i].color = sf::Color(200, 0, 200, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + (float)TILE_WIDTH * (startPos._x + 0.5f), 10.0f + (float)TILE_HEIGHT * (startPos._y + 0.5f));
	pathTiles2[pathLength2] = sf::Vector2f(10.0f + (float)TILE_WIDTH * (startPos2._x + 0.5f), 10.0f + (float)TILE_HEIGHT * (startPos2._y + 0.5f));
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
	delete[] walls;
	delete[] map;
	delete[] WallPos;
	ImGui::SFML::Shutdown();
	return 0;
}