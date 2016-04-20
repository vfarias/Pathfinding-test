#pragma once
#include "AStar.h"

class HPAStar : public Pathfinding
{
private:
	
	struct Cluster
	{
		Vec2D _position;
		HPANode** _internalNodes;		//pointers to all nodes within the cluster. All nodes are placed along the edges. cap = 2* _clusterSize
		int** _internalPathLengths;		//table of path length between each internal node
		int _nrOfInternalNodes;
		int _nodeCap;
		Cluster()
		{
			_position = {0,0};
			_internalNodes = nullptr;
			_internalPathLengths = nullptr;
			_nrOfInternalNodes = 0;
		}
		Cluster(Vec2D position, int maxInternalNodes)
		{
			_position = position;
			_nrOfInternalNodes = 0;
			_nodeCap = maxInternalNodes;
			_internalNodes = new HPANode*[_nodeCap];
			_internalPathLengths = new int*[_nodeCap];
			for (int i = 0; i < _nodeCap; i++)
			{
				_internalPathLengths[i] = new int[_nodeCap];
			}
		}
		~Cluster()
		{
			delete[] _internalNodes;
			for (int i = 0; i < _nodeCap; i++)
			{
				delete[] _internalPathLengths[i];
			}
			delete[] _internalPathLengths;
		}
	};
	BaseNode** _grid;
	int _clusterSize;				//Nr of tiles across in a cluster
	int _nrOfClusters;				//Total amount of clusters on the grid
	Cluster** _clusters;
	Heap<HPANode*> _openQueue;

	void setClusters();
	Cluster* findCluster(Vec2D position);		
	void findEdges(Vec2D pos, const Vec2D dir, Cluster* cluster1, Cluster* cluster2);				//dir = along the edge from pos. symm(t) = t + {dir.y, dir.x}
	void setEdgePair(HPANode* node1, HPANode* node2, Cluster* cluster1, Cluster* cluster2);
	void findInternalPaths(Cluster* cluster);
	int* attachNodeToGraph(HPANode* node);				//Used to fix start and goal nodes to the high level graph
	void calculateGCost(HPANode* parentNode, HPANode* childNode, int distance);
	void calculateHCost(HPANode* node);
public:
	HPAStar();
	HPAStar(int width, int height, int clusterSize, Heuristic heuristic = MANHATTAN);
	virtual ~HPAStar();
	void setTraversable(Vec2D pos, bool isTraversable = true);
	bool findPath(Metrics& metrics);
};