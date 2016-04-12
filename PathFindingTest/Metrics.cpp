#include "Metrics.h"
void Metrics::_expandNodeCap(Vec2D * arr, int & currentCap)
{
	int tempCap = currentCap * 2;
	Vec2D* temp = new Vec2D[tempCap];
	for (int i = 0; i < currentCap; i++)
	{
		temp[i] = arr[i];
	}
	delete[] arr;
	arr = temp;
	currentCap = tempCap;
}

Metrics::Metrics()
{
	_openedNodesCapacity = 100;
	_nrOfOpenedNodes = 0;
	_openedNodes = new Vec2D[_openedNodesCapacity];
	_expandedNodesCapacity = 100;
	_nrOfExpandedNodes = 0;
	_expandedNodes = new Vec2D[_expandedNodesCapacity];
}

Metrics::~Metrics()
{
	delete[] _openedNodes;
	_openedNodes = nullptr;
	delete[] _expandedNodes;
	_expandedNodes = nullptr;
}

int Metrics::getNrOfOpenedNodes() const
{
	return _nrOfOpenedNodes;
}

Vec2D * Metrics::getOpenedNodes() const
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

void Metrics::addOpenedNode(const Vec2D node)
{
	if (_nrOfOpenedNodes >= _openedNodesCapacity)
	{
		_expandNodeCap(_openedNodes, _openedNodesCapacity);
	}
	_openedNodes[_nrOfOpenedNodes++] = node;
}

void Metrics::addExpandedNode(const Vec2D node)
{
	if (_nrOfExpandedNodes >= _expandedNodesCapacity)
	{
		_expandNodeCap(_expandedNodes, _expandedNodesCapacity);
	}
	_expandedNodes[_nrOfExpandedNodes++] = node;
}
