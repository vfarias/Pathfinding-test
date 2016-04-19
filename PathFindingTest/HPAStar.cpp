#include "HPAStar.h"

void HPAStar::setClusters()
{
}

/*
Locates the cluster which contains the given position.
Returns the located cluster through reference.
*/
void HPAStar::findCluster(Vec2D position, Cluster* cluster)
{
	cluster = _clusters[position._y  * _height / (_clusterSize * _clusterSize) + position._x / _clusterSize];
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
	cluster1->_internalNodes[cluster1->_nrOfInternalNodes++] = node1;
	cluster2->_internalNodes[cluster2->_nrOfInternalNodes++] = node2;
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

void HPAStar::attachNodeToGraph(HPANode* node)
{
}

HPAStar::HPAStar()
{
}

HPAStar::HPAStar(int width, int height, int clusterSize, Heuristic heuristic)
	:Pathfinding(width, height, heuristic)
{
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
	attachNodeToGraph(start);
	HPANode* goal = new HPANode(_goal._x, _goal._y);
	attachNodeToGraph(goal);

	HPANode* currentNode = start;
	Cluster* currentCluster = nullptr;
	findCluster(start->_position, currentCluster);

	_nrOfPathNodes = 0;	
	Vec2D currentPos = _start;
	start->_open = 2;

	AStar* aStar = new AStar(_clusterSize, _clusterSize, EUCLIDEAN);				//Use A* to find path within clusters
	int* startToEdgePathLengths = new int[_clusterSize];
	for (int i = 0; i < currentCluster->_nrOfInternalNodes; i++)
	{
		aStar->init(_start, currentCluster->_internalNodes[i]->_position);
		for (int i = 0; i < _clusterSize; i++)
		{
			for (int j = 0; j < _clusterSize; j++)
			{
				int x = currentCluster->_position._x + j;
				int y = currentCluster->_position._y + i;
				aStar->setTraversable(Vec2D(x, y), _grid[x][y]._traversable);
			}
		}

	}	


	while (currentPos != _goal)														//loops until a path has been found
	{
		metrics.addExpandedNode(currentPos);
	}
	while (currentPos != _start)													//Count the path length to allocate enough memory for path
	{
		_nrOfPathNodes++;
		//currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
	_path = new Vec2D[_nrOfPathNodes];
	int c = 0;
	currentPos = _goal;
	while (currentPos != _start)													//Fill path
	{
		_path[c++] = currentPos;
		//currentPos = _grid[currentPos._x][currentPos._y]._parent->_position;
	}
		//Excluding start position since it should already be known
	return true;
}
