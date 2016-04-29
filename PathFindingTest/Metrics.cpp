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
	_pathLength = 0.0f;
	_nrOfPathNodes = 0;
	_pathNodesCapacity = 16;
	_pathNodes = new Vec2D[_pathNodesCapacity];
	_openedNodesCapacity = 64;
	_nrOfOpenedNodes = 0;
	_openedNodes = new Vec2D[_openedNodesCapacity];
	_expandedNodesCapacity = 64;
	_nrOfExpandedNodes = 0;
	_expandedNodes = new Vec2D[_expandedNodesCapacity];
	_graphNodesCapacity = 64;
	_nrOfGraphNodes = 0;
	_graphNodes = new Vec2D[_graphNodesCapacity];
}

Metrics::~Metrics()
{
	delete[] _pathNodes;
	_pathNodes = nullptr;
	delete[] _openedNodes;
	_openedNodes = nullptr;
	delete[] _expandedNodes;
	_expandedNodes = nullptr;
	delete[] _graphNodes;
	_graphNodes = nullptr;
}

float Metrics::getPathLength() const
{
	return _pathLength;
}

int Metrics::getNrOfPathNodes() const
{
	return _nrOfPathNodes;
}

Vec2D * Metrics::getPathNodes() const
{
	return _pathNodes;
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

void Metrics::addPathNode(const Vec2D node)
{
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

void Metrics::setPathNodes(Vec2D * path, int nrOfNodes)
{
	if (_pathNodes != nullptr)
	{
		delete[] _pathNodes;
	}
	_pathNodes = path;
	_pathNodesCapacity = nrOfNodes;
	_nrOfPathNodes = nrOfNodes;
	_pathLength = 0.0f;
	for (int i = 1; i < _nrOfPathNodes; i++)
	{
		float x = (float)(_pathNodes[i]._x - _pathNodes[i - 1]._x);
		float y = (float)(_pathNodes[i]._y - _pathNodes[i - 1]._y);
		_pathLength += std::sqrt(y * y + x * x);
	}
}

void Metrics::setPathNodes(Vec2D * path, int nrOfNodes, float pathLength)
{
	if (_pathNodes != nullptr)
	{
		delete[] _pathNodes;
	}
	_pathNodesCapacity = nrOfNodes;
	_nrOfPathNodes = nrOfNodes;
	_pathLength = pathLength;
	_pathNodes = new Vec2D[_pathNodesCapacity];
	for (int i = 0; i < _nrOfPathNodes; i++)
	{
		_pathNodes[i] = path[i];
	}
}

void Metrics::clean()
{
	_nrOfExpandedNodes = 0;
	_nrOfGraphNodes = 0;
	_nrOfOpenedNodes = 0;
	_nrOfPathNodes = 0;
}