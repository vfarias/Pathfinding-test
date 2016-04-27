#pragma once
#include "AStar.h"

class HPAStar : public Pathfinding
{
private:
	
	struct Cluster
	{
		Vec2D _position;
		HPANode** _internalNodes;		//pointers to all nodes within the cluster. All nodes are placed along the edges. cap = 2* _clusterSize
		float** _internalPathLengths;		//table of path length between each internal node
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
			_internalPathLengths = nullptr;
		}
		~Cluster()
		{
			for (int i = 0; i < _nrOfInternalNodes; i++)
			{
				delete[] _internalPathLengths[i];
				delete _internalNodes[i];
			}
			delete[] _internalPathLengths;
			delete[] _internalNodes;
		}
	};
	int _clusterSize;				//Nr of tiles across in a cluster
	int _nrOfClusters;				//Total amount of clusters on the grid
	Cluster** _clusters;
	Heap<HPANode*> _openQueue;

	void setClusters();
	Cluster* findCluster(Vec2D position);		
	void findEdges(Vec2D pos, const Vec2D dir, Cluster* cluster1, Cluster* cluster2, Metrics& metrics);		//dir = along the edge from pos. symm(t) = t + {dir.y, dir.x}
	void setEdgePair(Vec2D pos1, Vec2D pos2, Cluster* cluster1, Cluster* cluster2, Metrics& metrics);
	void findInternalPaths(Cluster* cluster, Metrics& metrics);
	float* attachNodeToGraph(HPANode* node, Metrics& metrics);				//Used to fix start and goal nodes to the high level graph
	void calculateGCost(HPANode* parentNode, HPANode* childNode, int distance);
	void calculateHCost(HPANode* node);
public:
	HPAStar();
	HPAStar(int width, int height, int clusterSize, AStarNode** grid, Heuristic heuristic = MANHATTAN);
	virtual ~HPAStar();
	bool findPath(Metrics& metrics);
	void cleanMap();
};