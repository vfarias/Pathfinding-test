#include "Metrics.h"
Vec2D* Metrics::_expandNodeCap(Vec2D* arr, int & currentCap)
{
	int tempCap = currentCap * 2;
	Vec2D* temp = new Vec2D[tempCap];

	for (int i = 0; i < currentCap; i++)
	{
		temp[i] = Vec2D(arr[i]._x, arr[i]._y);
	}
	int f = 0;
	delete[] arr;
	arr = temp;
	currentCap = tempCap;
	return arr;
}

Metrics::Metrics()
{
	_openedNodesCapacity = 4800;
	_nrOfOpenedNodes = 0;
	_openedNodes = new Vec2D[_openedNodesCapacity];
	_expandedNodesCapacity = 4800;
	_nrOfExpandedNodes = 0;
	_expandedNodes = new Vec2D[_expandedNodesCapacity];
	_graphNodesCapacity = 4800;
	_nrOfGraphNodes = 0;
	_graphNodes = new Vec2D[_graphNodesCapacity];
}

Metrics::~Metrics()
{
	delete[] _openedNodes;
	_openedNodes = nullptr;
	delete[] _expandedNodes;
	_expandedNodes = nullptr;
	delete[] _graphNodes;
	_graphNodes = nullptr;
}

int Metrics::getNrOfOpenedNodes() const
{
	return _nrOfOpenedNodes;
}

Vec2D* Metrics::getOpenedNodes() const
{
	return _openedNodes;
}

int Metrics::getNrOfExpandedNodes() const
{
	return _nrOfExpandedNodes;
}

Vec2D * Metrics::getExpandedNodes() const
{
	return _expandedNodes;
}

int Metrics::getNrOfGraphNodes() const
{
	return _nrOfGraphNodes;
}

Vec2D * Metrics::getGraphNodes() const
{
	return _graphNodes;
}

void Metrics::addOpenedNode(const Vec2D node)
{
	if (_nrOfOpenedNodes >= _openedNodesCapacity)
	{
		_openedNodes = _expandNodeCap(_openedNodes, _openedNodesCapacity);
	}
	_openedNodes[_nrOfOpenedNodes++] = node;
}

void Metrics::addExpandedNode(const Vec2D node)
{
	if (_nrOfExpandedNodes >= _expandedNodesCapacity)
	{
		_expandedNodes = _expandNodeCap(_expandedNodes, _expandedNodesCapacity);
	}
	_expandedNodes[_nrOfExpandedNodes++] = node;
}

void Metrics::addGraphNode(const Vec2D node)
{
	if (_nrOfGraphNodes >= _graphNodesCapacity)
	{
		_graphNodes = _expandNodeCap(_graphNodes, _graphNodesCapacity);
	}
	_graphNodes[_nrOfGraphNodes++] = node;
}
