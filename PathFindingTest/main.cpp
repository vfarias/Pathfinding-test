#include <SFML/Graphics.hpp>
#include "AStar.h"
#include "ThetaStar.h"
#include "Metrics.h"

int main()
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	int timer = 0;
	//Visual size of tiles
	const int TILE_WIDTH = 18;
	const int TILE_HEIGHT = 18;
	//Nr of tiles
	const int GRID_WIDTH = 12;			
	const int GRID_HEIGHT = 12;
	const int NR_OF_WALLS = 32;
	Vec2D startPos = {2, 8};
	Vec2D goalPos = {11, 11};
	Vec2D startPos2 = {1, 10};
	//AStar pathFinding(GRID_WIDTH, GRID_HEIGHT, startPos, goalPos, AStar::OCTILE, 1);
	ThetaStar pathFinding(GRID_WIDTH, GRID_HEIGHT, startPos, goalPos, ThetaStar::EUCLIDEAN);
	AStar pathFinding2(GRID_WIDTH, GRID_HEIGHT, startPos2, goalPos, AStar::OCTILE);
	int pathLength = 0;
	Vec2D* path = nullptr;

	Metrics metrics = Metrics();
	
	sf::RenderWindow window(sf::VideoMode(600, 500), "AI test");
	window.setFramerateLimit(2);
	//sf::CircleShape shape(100.f);
	//

	//sf::RectangleShape grid[GRID_WIDTH][GRID_HEIGHT];

	sf::CircleShape ai(0.4f * TILE_HEIGHT);
	ai.setPosition(sf::Vector2f(10 + startPos._x * TILE_WIDTH, 10 + startPos._y * TILE_HEIGHT));
	ai.setFillColor(sf::Color::Red);

	sf::CircleShape goal(0.4f * TILE_HEIGHT);
	//goal.setOutlineThickness(2.0);
	goal.setFillColor(sf::Color::Blue);
	goal.setPosition(sf::Vector2f(10 + goalPos._x * TILE_WIDTH, 10 + goalPos._y * TILE_HEIGHT));


	//2nd ai
	sf::CircleShape ai2(0.4f * TILE_HEIGHT);
	ai2.setPosition(sf::Vector2f(10 + startPos2._x * TILE_WIDTH, 10 + startPos2._y * TILE_HEIGHT));
	ai2.setFillColor(sf::Color::Green);



	for (int i = 0; i < GRID_WIDTH; i++)
	{
		for (int j = 0; j < GRID_HEIGHT; j++)
		{
			//grid[i][j] = sf::RectangleShape(sf::Vector2f(TILE_WIDTH, TILE_HEIGHT));
			//grid[i][j].setPosition(sf::Vector2f(10 + TILE_WIDTH * i, 10 + TILE_HEIGHT * j));
			//grid[i][j].setFillColor(sf::Color::White);
			//grid[i][j].setOutlineThickness(2.0);
			//grid[i][j].setOutlineColor(sf::Color::Black);
			pathFinding.setTraversable({i, j}, true);
		//	pathFinding2.setTileCost({i, j}, 1);
		}
	}
	

	//walls
	sf::RectangleShape walls[NR_OF_WALLS];
	Vec2D wallPos[NR_OF_WALLS] = {{2, 0},{2, 1},{2, 2},{2, 3},{3, 4},{4, 4},{5, 4},{8, 4},{9, 4},{10, 4},{7, 5},{0, 6},{1, 6},{2, 6},{3, 6},{5, 6},{7, 6},{9, 6},{5, 7},{7, 7},{9, 7},
	{4, 8},{7, 8},{9, 8},{3, 9},{7, 9},{9, 9},{2, 10},{7, 10},{9, 10},{1, 11},{9, 11}}; 

	for (int i = 0; i < NR_OF_WALLS; i++)
	{
		walls[i] = sf::RectangleShape(sf::Vector2f(TILE_WIDTH, TILE_HEIGHT));
		walls[i].setFillColor(sf::Color::White);
		walls[i].setPosition(sf::Vector2f(10 + TILE_WIDTH * wallPos[i]._x, 10 + TILE_HEIGHT * wallPos[i]._y));
		pathFinding.setTraversable(wallPos[i], false);
		pathFinding2.setTraversable(wallPos[i], false);
	}

	sf::RectangleShape* openedTiles = nullptr;
	sf::RectangleShape* expandedTiles = nullptr;
	sf::Vertex* pathTiles = nullptr;
	if (pathFinding.findPath(metrics))
	{
		pathLength = pathFinding.getNrOfPathNodes();
		path = pathFinding.getPath();
	}
	openedTiles = new sf::RectangleShape[metrics.getNrOfOpenedNodes()];
	for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
	{
		openedTiles[i] = sf::RectangleShape(sf::Vector2f(TILE_WIDTH, TILE_HEIGHT));
		openedTiles[i].setFillColor(sf::Color(0,0,200, 120));
		openedTiles[i].setPosition(sf::Vector2f(10 + TILE_WIDTH * metrics.getOpenedNodes()[i]._x, 10 + TILE_HEIGHT * metrics.getOpenedNodes()[i]._y));
	}

	
	expandedTiles = new sf::RectangleShape[metrics.getNrOfExpandedNodes()];
	for (int i = 0; i < metrics.getNrOfExpandedNodes(); i++)
	{
		expandedTiles[i] = sf::RectangleShape(sf::Vector2f(TILE_WIDTH, TILE_HEIGHT));
		expandedTiles[i].setFillColor(sf::Color(0, 200, 0, 120));
		expandedTiles[i].setPosition(sf::Vector2f(10 + TILE_WIDTH * metrics.getExpandedNodes()[i]._x, 10 + TILE_HEIGHT * metrics.getExpandedNodes()[i]._y));
	}

	pathTiles = new sf::Vertex[pathLength + 1];
	for (int i = 0; i < pathLength; i++)
	{
		pathTiles[i] = sf::Vertex(sf::Vector2f(10.0f + TILE_WIDTH * (path[i]._x + 0.5), 10.0f + TILE_HEIGHT * (path[i]._y + 0.5)));
		pathTiles[i].color = sf::Color(200, 200, 0, 255);
	}
	pathTiles[pathLength] = sf::Vector2f(10.0f + TILE_WIDTH * (startPos._x + 0.5), 10.0f + TILE_HEIGHT * (startPos._y + 0.5));
	//pathFinding2.findPath();
	//int pathLength2 = pathFinding2.getPathLength();
	//Vec2D* path2 = pathFinding2.getPath();

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}

		/*
			change goal
		*/
		//if (timer == 20)
		//{
		//	Vec2D goalPos = {0, 40};
		//	goal.setPosition(sf::Vector2f(10 + goalPos._x * TILE_WIDTH, 10 + goalPos._y * TILE_HEIGHT));
		//	pathFinding.setGoalPosition(goalPos);
		//	pathFinding.setStartPosition(path[pathLength + 1]);
		//	pathFinding.cleanMap();
		//	pathFinding.findPath();
		//	pathLength = pathFinding.getPathLength();
		//	path = pathFinding.getPath();
		//}

		/*
			change map
		*/
		//if (timer == 3)
		//{
		//	for (int i = 0; i < NR_OF_WALLS; i++)
		//	{
		//		pathFinding.setTileCost(wallPos[i], 1);
		//		pathFinding2.setTileCost(wallPos[i], 1);
		//	}

		//	for (int i = 0; i < NR_OF_WALLS; i++)
		//	{
		//		wallPos[i] = {3 + i, 3};
		//		walls[i].setPosition(sf::Vector2f(10 + TILE_WIDTH * wallPos[i]._x, 10 + TILE_HEIGHT * wallPos[i]._y));
		//		pathFinding.setTileCost(wallPos[i], -1);
		//		pathFinding2.setTileCost(wallPos[i], -1);
		//	}
		//	//ai.setPosition(sf::Vector2f(10 + path[pathLength + 1]._x * TILE_WIDTH, 10 + path[pathLength + 1]._y * TILE_HEIGHT));
		//	//Vec2D pos = path[pathLength];
		//	pathFinding.setStartPosition(path[pathLength - 1]);
		//	pathFinding.cleanMap();
		//	pathFinding.findPath();
		//	pathLength = pathFinding.getPathLength();
		//	path = pathFinding.getPath();


		//	//ai2.setPosition(sf::Vector2f(10 + path2[pathLength2 + 1]._x * TILE_WIDTH, 10 + path2[pathLength2 + 1]._y * TILE_HEIGHT));
		//	pathFinding2.setStartPosition(path2[pathLength2 - 1]);
		//	pathFinding2.cleanMap();
		//	pathFinding2.findPath();
		//	pathLength2 = pathFinding2.getPathLength();
		//	path2 = pathFinding2.getPath();
		//}

		/*if (pathLength > 0)
		{
			timer++;
			pathLength--;
			ai.setPosition(sf::Vector2f(10 + path[pathLength]._x * TILE_WIDTH, 10 + path[pathLength]._y * TILE_HEIGHT));
		}*/
		//if (pathLength2 > 0)
		//{
		//	pathLength2--;
		//	ai2.setPosition(sf::Vector2f(10 + path2[pathLength2]._x * TILE_WIDTH, 10 + path2[pathLength2]._y * TILE_HEIGHT));
		//}

		window.clear();

		//for (int i = 0; i < GRID_WIDTH; i++)
		//{
		//	for (int j = 0; j < GRID_HEIGHT; j++)
		//	{
		//		window.draw(grid[i][j]);
		//	}
		//}
		
		//window.draw(ai2);
		window.draw(goal);
		for (int i = 0; i < NR_OF_WALLS; i++)
		{
			window.draw(walls[i]);
		}
		for (int i = 0; i < metrics.getNrOfOpenedNodes(); i++)
		{
			window.draw(openedTiles[i]);
		}
		for (int i = 0; i <  metrics.getNrOfExpandedNodes(); i++)
		{
			window.draw(expandedTiles[i]);
		}

		window.draw(pathTiles, pathLength + 1, sf::LinesStrip);

	//	window.draw(ai);
		window.display();
	}
	//pathFinding.cleanMap();
	delete[] expandedTiles;
	delete[] openedTiles;
	delete[] pathTiles;
	return 0;
}