#include "HPAStar.h"

void HPAStar::setClusters()
{
}

/*
Locates the cluster which contains the given position.
Returns the located cluster through reference.
*/
HPAStar::Cluster* HPAStar::findCluster(Vec2D position)
{
	return _clusters[position._y  * _height / (_clusterSize * _clusterSize) + position._x / _clusterSize];
}

void HPAStar::findEdges(Vec2D pos, const Vec2D dir, Cluster* cluster1, Cluster* cluster2)
{
	int edgeLength = 0;						//This is the length of an open border between the clusters
	for (int i = 0; i < _clusterSize; i++)
	{
		if (_grid[pos._x][pos._y]._traversable)
		{
			edgeLength++;
		}
		else
		{
			if (edgeLength > 5)			//Cut off for an edge being long enough to get two node crossings.
			{
				HPANode* node1 = new HPANode(pos._x - dir._x, pos._y - dir._y);
				HPANode* node2 = new HPANode(pos._x - dir._x - dir._y, pos._y - dir._y - dir._x);
				setEdgePair(node1, node2, cluster1, cluster2);

				node1 = new HPANode(pos._x - dir._x * edgeLength, pos._y - dir._y * edgeLength);
				node2 = new HPANode(node1->_position._x - dir._y, node1->_position._y - dir._x);
				setEdgePair(node1, node2, cluster1, cluster2);
			}
			else if(edgeLength > 0)
			{
				HPANode* node1 = new HPANode(pos._x - dir._x * (edgeLength + 1) / 2, pos._y - dir._y * (edgeLength + 1) / 2);
				HPANode* node2 = new HPANode(pos._x - dir._y - dir._x * (edgeLength + 1) / 2, pos._y - dir._x - dir._y *(edgeLength + 1) / 2);
				setEdgePair(node1, node2, cluster1, cluster2);
			}
			edgeLength = 0;
		}
		pos += dir;
	}
	if (edgeLength > 5)					//Extra check in case the corner node is traversable
	{
		HPANode* node1 = new HPANode(pos._x - dir._x, pos._y - dir._y);
		HPANode* node2 = new HPANode(pos._x - dir._x - dir._y, pos._y - dir._y - dir._x);
		setEdgePair(node1, node2, cluster1, cluster2);

		node1 = new HPANode(pos._x - dir._x * edgeLength, pos._y - dir._y * edgeLength);
		node2 = new HPANode(node1->_position._x - dir._y, node1->_position._y - dir._x);
		setEdgePair(node1, node2, cluster1, cluster2);
	} else if (edgeLength > 0)
	{
		HPANode* node1 = new HPANode(pos._x - dir._x * (edgeLength + 1) / 2, pos._y - dir._y * (edgeLength + 1) / 2);
		HPANode* node2 = new HPANode(pos._x - dir._y - dir._x * (edgeLength + 1) / 2, pos._y - dir._x - dir._y *(edgeLength + 1) / 2);
		setEdgePair(node1, node2, cluster1, cluster2);
	}
}

void HPAStar::setEdgePair(HPANode* node1, HPANode* node2, Cluster* cluster1, Cluster* cluster2)
{
	node1->_edge = node2;
	node2->_edge = node1;
	cluster1->_internalNodes[cluster1->_nrOfInternalNodes] = node1;
	cluster2->_internalNodes[cluster2->_nrOfInternalNodes] = node2;
	node1->_clusterIndex = cluster1->_nrOfInternalNodes;
	node2->_clusterIndex = cluster2->_nrOfInternalNodes;
	cluster1->_nrOfInternalNodes++;
	cluster2->_nrOfInternalNodes++;
}

void HPAStar::findInternalPaths(Cluster* cluster)
{
	AStar* aStar = new AStar(_clusterSize, _clusterSize, _heuristicType);
	for (int i = 0; i < _clusterSize; i++)
	{
		for (int j = 0; j < _clusterSize; j++)
		{
			int x = cluster->_position._x + j;
			int y = cluster->_position._y + i;
			aStar->setTraversable(Vec2D(x, y), _grid[x][y]._traversable);
		}
	}
	for (int i = 0; i < cluster->_nrOfInternalNodes; i++)
	{
		for (int j = 0; j < cluster->_nrOfInternalNodes; j++)
		{
			if (i != j)
			{
				aStar->init(cluster->_internalNodes[i]->_position, cluster->_internalNodes[j]->_position);
				if (aStar->findPath(_metrics))
				{
					if (cluster->_internalPathLengths[i][j] < 0 || aStar->getNrOfPathNodes() <  cluster->_internalPathLengths[i][j])
					{
						cluster->_internalPathLengths[i][j] = aStar->getNrOfPathNodes();
					}
				}
			}
		}
	}
}

int* HPAStar::attachNodeToGraph(HPANode* node)
{
	AStar* aStar = new AStar(_clusterSize, _clusterSize, EUCLIDEAN);				//Use A* to find path within clusters
	Cluster* cluster = findCluster(node->_position);
	for (int i = 0; i < _clusterSize; i++)
	{
		for (int j = 0; j < _clusterSize; j++)
		{
			int x = cluster->_position._x + j;
			int y = cluster->_position._y + i;
			aStar->setTraversable(Vec2D(x, y), _grid[x][y]._traversable);
		}
	}
	int* nodeToEdgePathLengths = new int[cluster->_nrOfInternalNodes];

	for (int i = 0; i < cluster->_nrOfInternalNodes; i++)					//Get path lengths from start to cluster edges
	{
		aStar->init(_start, cluster->_internalNodes[i]->_position);
		if (aStar->findPath(_metrics))
		{
			nodeToEdgePathLengths[i] = aStar->getNrOfPathNodes();
		} else
		{
			nodeToEdgePathLengths[i] = -1;
		}
	}
	delete aStar;
	return nodeToEdgePathLengths;
}

void HPAStar::calculateGCost(HPANode * parentNode, HPANode * childNode, int distance)
{
}

void HPAStar::calculateHCost(HPANode* node)
{
	node->_hCost = getHeuristicDistance(node->_position, _goal);
}

HPAStar::HPAStar()
{
}

HPAStar::HPAStar(int width, int height, int clusterSize, Heuristic heuristic)
	:Pathfinding(width, height, heuristic)
{
	_openQueue = Heap<HPANode*>();
	_clusterSize = clusterSize;
	_nrOfClusters = _width * _height / (_clusterSize * _clusterSize);
	_clusters = new Cluster*[_nrOfClusters];
	for (int i = 0; i < _nrOfClusters; i++)
	{
		_clusters[i] = new Cluster(Vec2D((i * _clusterSize)%_width, (i * _clusterSize) / _width), _clusterSize * 2);
	}
	_grid = new BaseNode*[_width];
	for (int i = 0; i < _width; i++)
	{
		_grid[i] = new BaseNode[_height];
		for (int j = 0; j < _height; j++)
		{
			_grid[i][j] = {{i,j}, true};
		}
	}
}

HPAStar::~HPAStar()
{
	for (int i = 0; i < _nrOfClusters; i++)
	{
		delete _clusters[i];
	}
	delete[] _clusters;
	_clusters = nullptr;
	for (int i = 0; i < _width; i++)
	{
		delete[] _grid[i];
	}
	delete[] _grid;
}

void HPAStar::setTraversable(Vec2D pos, bool isTraversable)
{
	_grid[pos._x][pos._y]._traversable = isTraversable;
}

bool HPAStar::findPath(Metrics & metrics)
{
	if (_goal == _start)
	{
		return false;
	}
	for (int i = 1; i < _nrOfClusters; i++)
	{
		if (_clusters[i]->_position._x != 0)
		{
			findEdges(_clusters[i]->_position, {0, 1}, _clusters[i], _clusters[i - 1]);
		}
		if (_clusters[i]->_position._y != 0)
		{
			findEdges(_clusters[i]->_position, {1, 0}, _clusters[i], _clusters[i *_clusterSize - _width]);
		}
	}
	HPANode* start = new HPANode(_start._x, _start._y);
	int* startToEdgePathLengths = attachNodeToGraph(start);
	HPANode* goal = new HPANode(_goal._x, _goal._y);
	int* goalToEdgePathLengths = attachNodeToGraph(goal);

	HPANode* currentNode = start;
	Cluster* currentCluster = findCluster(start->_position);

	_nrOfPathNodes = 0;	
	start->_open = 2;


	for (int i = 0; i < currentCluster->_nrOfInternalNodes; i++)					//Get path lengths from start to cluster edges
	{
		if (startToEdgePathLengths[i] > 0)
		{
			HPANode* checkedNode = currentCluster->_internalNodes[i];
			checkedNode->_hCost = getHeuristicDistance(checkedNode->_position, _goal);
			checkedNode->_gCost = startToEdgePathLengths[i];
			checkedNode->_parent = currentNode;
			_openQueue.insert(checkedNode);
		}
	}

	currentNode = _openQueue.removeMin();
	currentNode->_open = 2;
	while (currentNode->_position != _goal)														//loops until a path has been found
	{
		metrics.addExpandedNode(currentNode->_position);

		if (currentCluster == findCluster(_goal) && goalToEdgePathLengths[currentNode->_clusterIndex] > 0)							//Check if the current node is linked to the goal
		{
			goal->_gCost = currentNode->_gCost + goalToEdgePathLengths[currentNode->_clusterIndex];
			_openQueue.insert(goal);
		}

		for (int i = 0; i < currentCluster->_nrOfInternalNodes; i++)
		{
			if (currentCluster->_internalPathLengths[currentNode->_clusterIndex][i] > 0)
			{
				HPANode* checkedNode = currentCluster->_internalNodes[i];
				float g = currentNode->_gCost + currentCluster->_internalPathLengths[currentNode->_clusterIndex][i];
				if (checkedNode->_open == 0 || checkedNode->_gCost > g)					//Add node to open
				{
					checkedNode->_hCost = getHeuristicDistance(checkedNode->_position, _goal);
					checkedNode->_gCost = g;
					checkedNode->_parent = currentNode;
					checkedNode->_open = 1;
					_openQueue.insert(checkedNode);
				}
			}
			if (currentNode->_edge->_open == 0 || currentNode->_edge->_gCost > currentNode->_gCost + 1)
			{
				currentNode->_edge->_hCost = getHeuristicDistance(currentNode->_edge->_position, _goal);
				currentNode->_edge->_gCost = currentNode->_gCost + 1;
				currentNode->_edge->_parent = currentNode;
				currentNode->_edge->_open = 1;
				_openQueue.insert(currentNode->_edge);
			}
		}
		if (_openQueue.size() <= 0)
		{
			return false;
		}
		currentNode = _openQueue.removeMin();
		currentNode->_open = 2;
		currentCluster = findCluster(currentNode->_position);
	}

	while (currentNode->_position != _start)													//Count the path length to allocate enough memory for path
	{
		_nrOfPathNodes++;
		//currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	_path = new Vec2D[_nrOfPathNodes];
	int c = 0;
	currentNode->_position = _goal;
	while (currentNode->_position != _start)													//Fill path
	{
		_path[c++] = currentNode->_position;
		//currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
		//Excluding start position since it should already be known
	return true;
}
