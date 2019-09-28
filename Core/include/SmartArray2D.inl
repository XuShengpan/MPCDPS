/*
**********************************************************************
*
* This file is a part of library MPCDPS(Massive Point Cloud Data Processing System).
* It is a free program and it is protected by the license GPL-v3.0, you may not use the
* file except in compliance with the License.
*
* Copyright(c) 2013 - 2019 Xu Shengpan, all rights reserved.
*
* Email: jack_1227x@163.com
*
**********************************************************************
*/

template <typename T, uint WIDTH>
SmartArray2D< T, WIDTH>::SmartArray2D(void) :_data(NULL), _elem_num(0)
{

}

template <typename T, uint WIDTH>
SmartArray2D< T, WIDTH>::~SmartArray2D(void)
{
	clear();
}

template <typename T, uint WIDTH>
SmartArray2D< T, WIDTH>::SmartArray2D(uint length, T* data)
{
	_elem_num = length;
	_data = data;
	if (data == NULL && length > 0) {
		_data = new T[length * WIDTH];
	}

	if (_data) {
		RefManager::getInstance()->referenceAdd(_data);
	}
}

template <typename T, uint WIDTH>
SmartArray2D< T, WIDTH>::SmartArray2D(const SmartArray2D< T, WIDTH>& rth)
	:_data(rth._data), _elem_num(rth._elem_num)
{
	if (_data) {
		RefManager::getInstance()->referenceAdd(_data);
	}
}

template <typename T, uint WIDTH>
void SmartArray2D< T, WIDTH>::operator = (const SmartArray2D< T, WIDTH>& rth)
{
	if (_data == rth.buffer()) {
		return;
	}
	clear();
	_data = rth.buffer();
	_elem_num = rth.size();
	if (_data) {
		RefManager::getInstance()->referenceAdd(_data);
	}
}

template <typename T, uint WIDTH>
SmartArray2D< T, WIDTH> SmartArray2D< T, WIDTH>::clone() const
{
	SmartArray2D< T, WIDTH> obj(_elem_num);
	if (_data) {
		memcpy(obj._data, _data, _elem_num * sizeof(T) * WIDTH);
	}
	return obj;
}

template <typename T, uint WIDTH>
int  SmartArray2D< T, WIDTH>::append(const SmartArray2D< T, WIDTH>& rth)
{
	int pos = -1;
	if (empty()) {
		_data = rth.buffer();
		_elem_num = rth.size();
		RefManager::getInstance()->referenceAdd(_data);
		pos = 0;
	}
	else {
		if (!rth.empty()) {
			int elem_n = _elem_num + rth._elem_num;
			pos = _elem_num;
			T* data = new T[elem_n * WIDTH];
			memcpy(data, _data, _elem_num * WIDTH * sizeof(T));
			memcpy(data + _elem_num * WIDTH, rth._data, rth._elem_num * WIDTH * sizeof(T));
			clear();
			_data = data;
			_elem_num = elem_n;
			RefManager::getInstance()->referenceAdd(_data);
		}
	}
	return pos;
}

template <typename T, uint WIDTH>
void SmartArray2D< T, WIDTH>::copyOut(uint beg, uint end, SmartArray2D< T, WIDTH>& copyObj)
{
	assert(beg >= 0 && beg < end && end <= _elem_num);
	int elem_n = end - beg;
	T* data = new T[elem_n * WIDTH];
	memcpy(data, _data + beg * WIDTH, sizeof(T) * elem_n * WIDTH);
	copyObj = SmartArray2D< T, WIDTH>(elem_n, data);
}


template <typename T, uint WIDTH>
void SmartArray2D< T, WIDTH>::copyIn(uint pos_dest, uint pos_src, uint elem_n, const SmartArray2D< T, WIDTH>& rth)
{
	assert(elem_n <= rth.size());
	if (_elem_num < pos_dest + elem_n) {
		resize(pos_dest + elem_n);
	}
	memcpy(_data + pos_dest * WIDTH, rth.buffer() + pos_src * WIDTH, elem_n * WIDTH * sizeof(T));
}

template <typename T, uint WIDTH>
void SmartArray2D< T, WIDTH>::clear()
{
	if (_data) {
		RefManager::getInstance()->referenceReduce2<T>(_data);
		_data = NULL;
		_elem_num = 0;
	}
}

template <typename T, uint WIDTH>
void SmartArray2D< T, WIDTH>::resize(uint length)
{
	if (length == _elem_num) return;
	if (length == 0) {
		clear();
		return;
	}

	T* data = new T[length * WIDTH];
	if (_elem_num > length) {
		memcpy(data, _data, length * WIDTH * sizeof(T));
	}
	else {
		memcpy(data, _data, _elem_num * WIDTH * sizeof(T));
	}
	clear();
	_elem_num = length;
	_data = data;
	RefManager::getInstance()->referenceAdd(_data);
	return;
}

template <typename T, uint WIDTH>
T* SmartArray2D< T, WIDTH>::operator[] (const uint i) const
{
	assert(i < _elem_num);
	return _data + i * WIDTH;
}

template <typename T, uint WIDTH>
T* SmartArray2D< T, WIDTH>::ptrElem(uint i) const
{
	assert(i < _elem_num);
	return _data + i * WIDTH;
}

template <typename T, uint WIDTH>
uint SmartArray2D< T, WIDTH>::size() const
{
	return _elem_num;
}

template <typename T, uint WIDTH>
T* SmartArray2D< T, WIDTH>::buffer() const
{
	return _data;
}

template <typename T, uint WIDTH>
bool SmartArray2D< T, WIDTH>::empty() const
{
	return _elem_num == 0;
}