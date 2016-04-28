#pragma once
#include "AIUtil.h"
class Metrics
{
private:
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

	Vec2D* _expandNodeCap( Vec2D* arr, int& currentCap);
public:
	Metrics();
	~Metrics();

	int getNrOfOpenedNodes() const;
	Vec2D* getOpenedNodes() const;
	int getNrOfExpandedNodes() const;
	Vec2D* getExpandedNodes() const;
	int getNrOfGraphNodes() const;
	Vec2D* getGraphNodes() const;

	void addOpenedNode(const Vec2D node);
	void addExpandedNode(const Vec2D node);
	void addGraphNode(const Vec2D node);
	void clean();
};