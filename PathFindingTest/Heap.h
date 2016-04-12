#pragma once

/*
Simple min-heap.
Needs < and > for comparison
--Victor
*/
template <typename T>
class Heap
{
private:
	T* _tree;
	int _capacity;
	int _nrOfElements;
public:
	Heap();
	//Heap(const Heap<T>& comp);
	virtual ~Heap();
	void operator=(const Heap<T>& comp);
	void insert(T element);
	T removeMin();
	T getMin()const;
	int size() const;
	void swap(T& obj1, T& obj2);
	void empty();
};

template<typename T>
Heap<T>::Heap()
{
	_capacity = 15;		//4  rows
	_nrOfElements = 0;
	_tree = new T[_capacity];
}


/*
template<typename T>
Heap<T>::Heap(const Heap<T>& comp)
{
*this = *comp;
}
*/


template<typename T>
Heap<T>::~Heap()
{
	delete[] _tree;
	_tree = nullptr;
}

template<typename T>
void Heap<T>::operator=(const Heap<T>& comp)
{
	delete[] _tree;

	_capacity = comp._capacity;
	_nrOfElements = comp._nrOfElements;
	_tree = new T[_capacity];
	for (int i = 0; i < _nrOfElements; i++)
	{
		_tree[i] = comp._tree[i];
	}
}

template<typename T>
void Heap<T>::insert(T element)
{
	if (_nrOfElements >= _capacity)
	{
		_capacity = _capacity * 2 + 1;
		T* temp = new T[_capacity];
		for (int i = 0; i < _nrOfElements; i++)
		{
			temp[i] = _tree[i];
		}
		delete[] _tree;
		_tree = temp;
	}
	int pos = _nrOfElements;
	_tree[_nrOfElements++] = element;
	while (_tree[pos] < _tree[(pos - 1) / 2])
	{
		swap(_tree[pos], _tree[(pos - 1) / 2]);
		pos = (pos - 1) / 2;
	}
}

template<typename T>
T Heap<T>::removeMin()
{
	int pos = 0;
	T result = _tree[0];
	swap(_tree[0], _tree[--_nrOfElements]);
	while (2 * pos + 1 < _nrOfElements && (_tree[pos] > _tree[2 * pos + 1] || (_tree[pos] > _tree[2 * pos + 2] && 2 * pos + 2 < _nrOfElements)))
	{
		if (_tree[2 * pos + 2] > _tree[2 * pos + 1] || 2 * pos + 2 >= _nrOfElements)
		{
			swap(_tree[pos], _tree[2 * pos + 1]);
			pos = 2 * pos + 1;
		}
		else
		{
			swap(_tree[pos], _tree[2 * pos + 2]);
			pos = 2 * pos + 2;
		}
	}
	return result;
}

template<typename T>
T Heap<T>::getMin()const
{
	return _tree[0];
}

template<typename T>
int Heap<T>::size() const
{
	return _nrOfElements;
}

template<typename T>
void Heap<T>::swap(T& obj1, T& obj2)
{
	T temp = obj1;
	obj1 = obj2;
	obj2 = temp;
}

template<typename T>
void Heap<T>::empty()
{
	_nrOfElements = 0;
}