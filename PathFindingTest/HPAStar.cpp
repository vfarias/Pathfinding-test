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
	return _clusters[(position._y / _clusterSize) * (_width / _clusterSize) + (position._x / _clusterSize)];
}

void HPAStar::findEdges(Vec2D pos, const Vec2D dir, Cluster* cluster1, Cluster* cluster2, Metrics& metrics)
{
	int edgeLength = 0;						//This is the length of an open border between the clusters
	for (int i = 0; i < _clusterSize; i++)
	{
		if (_grid[pos._x][pos._y]._traversable && _grid[pos._x - dir._y][pos._y - dir._x]._traversable)
		{
			edgeLength++;
		}
		else
		{
			if (edgeLength > 5)			//Cut off for an edge being long enough to get two node crossings.
			{
				Vec2D node1 = Vec2D(pos._x - dir._x, pos._y - dir._y);
				Vec2D node2 = Vec2D(node1._x - dir._y, node1._y - dir._x);
				setEdgePair(node1, node2, cluster1, cluster2, metrics);

				node1 = Vec2D(pos._x - dir._x * edgeLength, pos._y - dir._y * edgeLength);
				node2 = Vec2D(node1._x - dir._y, node1._y - dir._x);
				setEdgePair(node1, node2, cluster1, cluster2, metrics);
			}
			else if(edgeLength > 0)
			{
				Vec2D node1 = Vec2D(pos._x - dir._x * (edgeLength + 1) / 2, pos._y - dir._y * (edgeLength + 1) / 2);
				Vec2D node2 = Vec2D(node1._x - dir._y, node1._y - dir._x);
				setEdgePair(node1, node2, cluster1, cluster2, metrics);
			}
			edgeLength = 0;
		}
		pos += dir;
	}
	if (edgeLength > 5)					//Extra check in case the corner node is traversable
	{
		Vec2D node1 = Vec2D(pos._x - dir._x, pos._y - dir._y);
		Vec2D node2 = Vec2D(node1._x - dir._y, node1._y - dir._x);
		setEdgePair(node1, node2, cluster1, cluster2, metrics);

		node1 = Vec2D(pos._x - dir._x * edgeLength, pos._y - dir._y * edgeLength);
		node2 = Vec2D(node1._x - dir._y, node1._y - dir._x);
		setEdgePair(node1, node2, cluster1, cluster2, metrics);
	} else if (edgeLength > 0)
	{
		Vec2D node1 = Vec2D(pos._x - dir._x * (edgeLength + 1) / 2, pos._y - dir._y * (edgeLength + 1) / 2);
		Vec2D node2 = Vec2D(node1._x - dir._y, node1._y - dir._x);
		setEdgePair(node1, node2, cluster1, cluster2, metrics);
	}
}

void HPAStar::setEdgePair(Vec2D pos1, Vec2D pos2, Cluster* cluster1, Cluster* cluster2, Metrics& metrics)
{
	//HPANode* node1 = new HPANode(pos1._x, pos1._y);
	HPANode* node1 = nullptr;
	HPANode* node2 = nullptr;
	bool makeNode1 = false;
	if (pos1 == cluster1->_position && pos2 == pos1 + Vec2D(0, -1) &&
		isPositionValid({pos1._x - 1, pos1._y}) && _grid[pos1._x - 1][pos1._y]._traversable)		// 2nd edge for top left corner
	{
		for (int i = 0; i < cluster1->_nrOfInternalNodes && node1 == nullptr; i++)
		{
			if (pos1 == cluster1->_internalNodes[i]->_position)
			{
				node1 = cluster1->_internalNodes[i];
			}
		}
	}
	if (node1 == nullptr)
	{
		node1 = new HPANode(pos1._x, pos1._y);
		cluster1->_internalNodes[cluster1->_nrOfInternalNodes] = node1;
		node1->_clusterIndex = cluster1->_nrOfInternalNodes;
		cluster1->_nrOfInternalNodes++;
		makeNode1 = true;
	}

	bool makeNode2 = true;
	if (pos1 == (pos2 + Vec2D(0,1)) && pos2 == Vec2D(cluster2->_position._x + _clusterSize - 1, cluster2->_position._y + _clusterSize - 1))	//bottom right
	{
		if ( isPositionValid({pos2._x + 1, pos2._y}) && _grid[pos2._x + 1][pos2._y]._traversable)
		{
			makeNode2 = false;
		}
	}
	else if (pos2 == Vec2D(cluster2->_position._x + _clusterSize - 1, cluster2->_position._y) ||				//top right
			 pos2 == Vec2D(cluster2->_position._x, cluster2->_position._y + _clusterSize - 1))					//bottom left
	{
		Vec2D cornerPos = {pos2._x - (pos1._y - pos2._y), pos2._y - (pos1._x - pos2._x)};
		if (isPositionValid(cornerPos) && _grid[cornerPos._x][cornerPos._y]._traversable)
		{
			makeNode2 = false;
		}
	}
	if (!makeNode2)
	{
		for (int i = 0; i < cluster2->_nrOfInternalNodes && node2 == nullptr; i++)
		{
			if (pos2 == cluster2->_internalNodes[i]->_position)
			{
				node2 = cluster2->_internalNodes[i];
				node2->_edge2 = node1;
			}
		}
	}
	if (node2 == nullptr)
	{
		node2 = new HPANode(pos2._x, pos2._y);
		cluster2->_internalNodes[cluster2->_nrOfInternalNodes] = node2;
		node2->_clusterIndex = cluster2->_nrOfInternalNodes;
		cluster2->_nrOfInternalNodes++;
		node2->_edge = node1;
	}
	if (node1->_edge == nullptr)
	{
		node1->_edge = node2;
	}
	else
	{
		node1->_edge2 = node2;
	}

	metrics.addGraphNode(pos1);
	metrics.addGraphNode(pos2);
}

void HPAStar::findInternalPaths(Cluster* cluster, Metrics& metrics)
{
	cluster->_internalPathLengths = new float*[cluster->_nrOfInternalNodes];
	for (int i = 0; i <  cluster->_nrOfInternalNodes; i++)
	{
		cluster->_internalPathLengths[i] = new float[cluster->_nrOfInternalNodes];
		for (int j = 0; j < cluster->_nrOfInternalNodes; j++)
		{
			cluster->_internalPathLengths[i][j] = -1;
		}
	}
	AStar* aStar = new AStar(_clusterSize, _clusterSize, cluster->_position, _grid, _heuristicType);
	//for (int i = 0; i < _clusterSize; i++)
	//{
	//	for (int j = 0; j < _clusterSize; j++)
	//	{
	//		aStar->setTraversable(Vec2D(j, i), _grid[cluster->_position._x + j][cluster->_position._y + i]._traversable);
	//	}
	//}
	for (int i = 0; i < cluster->_nrOfInternalNodes; i++)
	{
		for (int j = i + 1; j < cluster->_nrOfInternalNodes; j++)
		{
			
			bool possiblePath = true;
			for (int k = 0; k < i && possiblePath; k++)
			{
				if ((cluster->_internalPathLengths[k][i] > 0 && cluster->_internalPathLengths[k][j] < 0) ||
					(cluster->_internalPathLengths[k][i] < 0 && cluster->_internalPathLengths[k][j] > 0))			//There can't be a path
				{
					possiblePath = false;
				}
				else if ((cluster->_internalPathLengths[k][i] > 0 && cluster->_internalPathLengths[k][j] > 0))		//There is a guaranteed path
				{
					k = i;
				}
			}
			if (possiblePath)
			{
				aStar->init(cluster->_internalNodes[i]->_position, cluster->_internalNodes[j]->_position);
				if (aStar->findPath(metrics))
				{
					if (cluster->_internalPathLengths[i][j] < 0 || aStar->getPathLength() < cluster->_internalPathLengths[i][j])
					{
						cluster->_internalPathLengths[i][j] = aStar->getPathLength();
						cluster->_internalPathLengths[j][i] = aStar->getPathLength();
						metrics.addGraphNode(cluster->_internalNodes[i]->_position);
						metrics.addGraphNode(cluster->_internalNodes[j]->_position);
					}
				}
			}
		}
	}
	delete aStar;
}

float* HPAStar::attachNodeToGraph(HPANode* node, Metrics& metrics)
{
	Cluster* cluster = findCluster(node->_position);
	AStar* aStar = new AStar(_clusterSize, _clusterSize, cluster->_position, _grid, OCTILE);				//Use A* to find path within clusters
	//for (int i = 0; i < _clusterSize; i++)
	//{
	//	for (int j = 0; j < _clusterSize; j++)
	//	{
	//		aStar->setTraversable(Vec2D(j, i), _grid[cluster->_position._x + j][cluster->_position._y + i]._traversable);
	//	}
	//}
	float* nodeToEdgePathLengths = new float[cluster->_nrOfInternalNodes];

	for (int i = 0; i < cluster->_nrOfInternalNodes; i++)					//Get path lengths from start to cluster edges
	{
		aStar->init(node->_position, cluster->_internalNodes[i]->_position);
		if (aStar->findPath(metrics))
		{
			nodeToEdgePathLengths[i] = aStar->getPathLength();
			metrics.addGraphNode(cluster->_internalNodes[i]->_position);
			metrics.addGraphNode(node->_position);
		} else
		{
			nodeToEdgePathLengths[i] = -1.0f;
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
	:Pathfinding()
{
	_openQueue = Heap<HPANode*>();
	_clusters = nullptr;
}

HPAStar::HPAStar(int width, int height, int clusterSize, AStarNode** grid, Heuristic heuristic)
	:Pathfinding(width, height, grid, {0,0}, heuristic)
{
	_openQueue = Heap<HPANode*>();
	_clusterSize = clusterSize;
	_nrOfClusters = _width * _height / (_clusterSize * _clusterSize);
	_clusters = new Cluster*[_nrOfClusters];
	for (int i = 0; i < _nrOfClusters; i++)
	{
		_clusters[i] = new Cluster(Vec2D((i * _clusterSize)%_width, _clusterSize *  (i/ (_width / _clusterSize))), _clusterSize * 2);
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
}

bool HPAStar::findPath(Metrics& metrics)
{
	if (_goal == _start)
	{
		return false;
	}
	for (int i = 1; i < _nrOfClusters; i++)
	{
		if (_clusters[i]->_position._x != 0)
		{
			findEdges(_clusters[i]->_position, {0, 1}, _clusters[i], _clusters[i - 1], metrics);
		}
		if (_clusters[i]->_position._y != 0)
		{
			findEdges(_clusters[i]->_position, {1, 0}, _clusters[i], _clusters[i - _width / _clusterSize], metrics);
		}
	}
	for (int i = 0; i < _nrOfClusters; i++)
	{
		findInternalPaths(_clusters[i], metrics);
	}
	HPANode start = HPANode(_start._x, _start._y);
	start._gCost = 0;
	float* startToEdgePathLengths = attachNodeToGraph(&start, metrics);
	HPANode goal = HPANode(_goal._x, _goal._y);
	goal._hCost = 0;
	float* goalToEdgePathLengths = attachNodeToGraph(&goal, metrics);

	HPANode* currentNode = &start;
	Cluster* currentCluster = findCluster(start._position);

	_nrOfPathNodes = 0;	
	start._open = 2;


	for (int i = 0; i < currentCluster->_nrOfInternalNodes; i++)					//Get path lengths from start to cluster edges
	{
		if (startToEdgePathLengths[i] > 0)
		{
			HPANode* checkedNode = currentCluster->_internalNodes[i];
			checkedNode->_hCost = getHeuristicDistance(checkedNode->_position, _goal);
			checkedNode->_gCost = startToEdgePathLengths[i];
			checkedNode->_parent = currentNode;
			_openQueue.insert(checkedNode);
			checkedNode->_open = 1;
			metrics.addOpenedNode(checkedNode->_position);
			metrics.addOpenedNode(currentNode->_position);
		}
	}

	if (_openQueue.size() <= 0)
	{
		return false;
	}
	currentNode = _openQueue.removeMin();
	currentNode->_open = 2;
	metrics.addExpandedNode(currentNode->_parent->_position);
	metrics.addExpandedNode(currentNode->_position);
	while (currentNode->_position != _goal)														//loops until a path has been found
	{
		//metrics.addExpandedNode(currentNode->_position);

		if (currentCluster == findCluster(_goal) && goalToEdgePathLengths[currentNode->_clusterIndex] > 0)							//Check if the current node is linked to the goal
		{
			goal._gCost = currentNode->_gCost + goalToEdgePathLengths[currentNode->_clusterIndex];
			goal._parent = currentNode;
			goal._open = 1;
			_openQueue.insert(&goal);
		}

		for (int i = 0; i < currentCluster->_nrOfInternalNodes; i++)
		{
			if (currentCluster->_internalPathLengths[currentNode->_clusterIndex][i] > 0)
			{
				HPANode* checkedNode = currentCluster->_internalNodes[i];
				metrics.addOpenedNode(checkedNode->_position);
				metrics.addOpenedNode(currentNode->_position);
				float g = currentNode->_gCost + currentCluster->_internalPathLengths[currentNode->_clusterIndex][i];
				if (checkedNode->_open == 0 ||(checkedNode->_open == 1 && checkedNode->_gCost > g))					//Add node to open
				{
					checkedNode->_hCost = getHeuristicDistance(checkedNode->_position, _goal);
					checkedNode->_gCost = g;
					checkedNode->_parent = currentNode;
					checkedNode->_open = 1;
					_openQueue.insert(checkedNode);
					
				}
			}
		}
		if (currentNode->_edge->_open == 0 || (currentNode->_edge->_open == 1 && currentNode->_edge->_gCost > currentNode->_gCost + 1))
		{
			currentNode->_edge->_hCost = getHeuristicDistance(currentNode->_edge->_position, _goal);
			currentNode->_edge->_gCost = currentNode->_gCost + 1;
			currentNode->_edge->_parent = currentNode;
			currentNode->_edge->_open = 1;
			_openQueue.insert(currentNode->_edge);
			metrics.addOpenedNode(currentNode->_edge->_position);
			metrics.addOpenedNode(currentNode->_position);
		}
		if (currentNode->_edge2 != nullptr && (currentNode->_edge2->_open == 0 || (currentNode->_edge2->_open == 1 && currentNode->_edge2->_gCost > currentNode->_gCost + 1)))
		{
			currentNode->_edge2->_hCost = getHeuristicDistance(currentNode->_edge2->_position, _goal);
			currentNode->_edge2->_gCost = currentNode->_gCost + 1;
			currentNode->_edge2->_parent = currentNode;
			currentNode->_edge2->_open = 1;
			_openQueue.insert(currentNode->_edge2);
			metrics.addOpenedNode(currentNode->_edge2->_position);
			metrics.addOpenedNode(currentNode->_position);
		}
		if (_openQueue.size() <= 0)
		{
			//return false;
			_goal = currentNode->_position;
		}
		else
		{
			currentNode = _openQueue.removeMin();
			while (currentNode->_open == 2)
			{
				if (_openQueue.size() <= 0)
				{
					return false;
				}
				currentNode = _openQueue.removeMin();
			}
			metrics.addExpandedNode(currentNode->_parent->_position);
			metrics.addExpandedNode(currentNode->_position);
			currentNode->_open = 2;
			currentCluster = findCluster(currentNode->_position);
		}
	}
	AStar* aStar = new AStar(_clusterSize, _clusterSize, currentCluster->_position, _grid, OCTILE);
	_path = new Vec2D[currentNode->_gCost];
	while (currentNode->_position != _start)							//Count the path length to allocate enough memory for path
	{
		if (currentNode->_edge != nullptr && currentNode->_edge->_position == currentNode->_parent->_position)
		{
			_path[_nrOfPathNodes++] = currentNode->_parent->_position;
		}
		else
		{
			aStar->setPosition(currentCluster->_position);
			aStar->init(currentNode->_position, currentNode->_parent->_position);
			//for (int i = 0; i < _clusterSize; i++)					//TODO: See if the copying can be changed to just pointing towards the general grid
			//{
			//	for (int j = 0; j < _clusterSize; j++)
			//	{
			//		aStar->setTraversable(Vec2D(j, i), _grid[currentCluster->_position._x + j][currentCluster->_position._y + i]._traversable);
			//	}
			//}
			if (aStar->findPath(metrics))
			{
				Vec2D* tempPath = aStar->getPath();
				for (int i = aStar->getNrOfPathNodes(); i > 0; i--)					//TODO: See if the copying can be changed to just pointing towards the general grid
				{
					_path[_nrOfPathNodes++] = tempPath[i - 1];
				}
			}
		}
		currentNode = currentNode->_parent;
		currentCluster = findCluster(currentNode->_position);
	}
	metrics.setPathNodes(_path, _nrOfPathNodes, _grid[_goal._x][_goal._y]._gCost);
	delete aStar;
	delete startToEdgePathLengths;
	delete goalToEdgePathLengths;
	return true;
}

void HPAStar::cleanMap()
{
	Pathfinding::cleanMap();

	for (__int16 i = _position._x; i < _position._x + _width; i++)
	{
		for (__int16 j = _position._y; j < _position._y + _height; j++)
		{
			_grid[i][j]._open = 0;
			_grid[i][j]._gCost = 0;
			_grid[i][j]._hCost = 0;
			_grid[i][j]._parent = nullptr;
		}
	}
	_openQueue.empty();
	_openQueue = Heap<HPANode*>();
}
