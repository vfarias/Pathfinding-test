#pragma once
#include "AIUtil.h"
#include <algorithm>
class Metrics
{
private:
	float _pathLength;
	int _pathNodesCapacity;
	int _nrOfPathNodes;
	Vec2D* _pathNodes;
	int _openedNodesCapacity;
	int _nrOfOpenedNodes;
	Vec2D* _openedNodes;					// The nodes added to the pathfinding graph
	int _expandedNodesCapacity;
	int _nrOfExpandedNodes;				
	Vec2D* _expandedNodes;					// The nodes which have added all their children to the pathfinding graph
	//TODO: Add path / path length
	int _graphNodesCapacity;
	int _nrOfGraphNodes;
	Vec2D* _graphNodes;
	long _expansionCounter;					//counts the expanded nodes, but doesn't keep track of positions

	Vec2D* _expandNodeCap( Vec2D* arr, int& currentCap);
public:
	Metrics();
	~Metrics();

	float getPathLength() const;
	int getNrOfPathNodes() const;
	Vec2D* getPathNodes() const;
	int getNrOfOpenedNodes() const;
	Vec2D* getOpenedNodes() const;
	int getNrOfExpandedNodes() const;
	Vec2D* getExpandedNodes() const;
	int getNrOfGraphNodes() const;
	Vec2D* getGraphNodes() const;

	void addPathNode(const Vec2D node);
	void addOpenedNode(const Vec2D node);
	void addExpandedNode(const Vec2D node);
	void addGraphNode(const Vec2D node);
	void countExpansion();

	void setPathNodes(Vec2D* path, int nrOfNodes);
	void setPathNodes(Vec2D* path, int nrOfNodes, float pathLength);
	void clean();
};