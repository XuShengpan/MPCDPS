/*
**********************************************************************
*
* This file is a part of library MPCDPS(Massive Point Cloud Data Processing System).
* It is a free program and it is protected by the license GPL-v3.0, you may not use the
* file except in compliance with the License.
*
* Copyright(c) 2013 - 2020 Xu Shengpan, all rights reserved.
*
* Email: jack_1227x@163.com
*
**********************************************************************
*/

template <typename T>
SmartArray<T>::SmartArray(void) :_elem_num(0), _data(NULL)
{
}

template <typename T>
SmartArray<T>::SmartArray(uint elem_n, T* data) : _elem_num(elem_n), _data(data)
{
	if (_elem_num > 0 && data == NULL) {
		_data = new T[elem_n];
	}

	RefManager::getInstance()->referenceAdd(_data);
}

template <typename T>
SmartArray<T>::SmartArray(const SmartArray<T>& rth)
	:_elem_num(rth._elem_num), _data(rth._data)
{
	RefManager::getInstance()->referenceAdd(_data);
}

template <typename T>
SmartArray<T>::~SmartArray(void)
{
	clear();
}

template <typename T>
SmartArray<T> SmartArray<T>::clone() const
{
	SmartArray<T> obj(_elem_num);
	if (_data) {
		memcpy(obj._data, _data, _elem_num * sizeof(T));
	}
	return obj;
}

template <typename T>
int  SmartArray<T>::append(const SmartArray<T>& rth)
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
			T* data = new T[elem_n];
			memcpy(data, _data, _elem_num * sizeof(T));
			memcpy(data + _elem_num, rth._data, rth._elem_num * sizeof(T));
			clear();
			_data = data;
			_elem_num = elem_n;
			RefManager::getInstance()->referenceAdd(_data);
		}
	}
	return pos;
}

template <typename T>
void SmartArray<T>::operator = (const SmartArray<T> & rth)
{
	if (_data == rth.buffer()) {
		return;
	}

	if (_data) {
		clear();
	}

	_data = rth.buffer();
	_elem_num = rth.size();
	RefManager::getInstance()->referenceAdd(_data);
}

template <typename T>
void SmartArray<T>::reset(const T& t)
{
	for (uint i = 0; i < _elem_num; ++i) {
		_data[i] = t;
	}
}

template <typename T>
T& SmartArray<T>::operator[] (const uint i)
{
	assert(i < _elem_num);
	return _data[i];
}

template <typename T>
const T& SmartArray<T>::operator[] (const uint i) const
{
	assert(i < _elem_num);
	return _data[i];
}

template <typename T>
T* SmartArray<T>::ptrElem(const uint i) const
{
	assert(i < _elem_num);
	return _data + i;
}

template <typename T>
uint SmartArray<T>::size() const
{
	return _elem_num;
}

template <typename T>
T* SmartArray<T>::buffer() const
{
	return _data;
}

template <typename T>
void SmartArray<T>::copyOut(uint beg, uint end, SmartArray<T>& copyObj)
{
	assert(beg >= 0 && beg < end && end <= _elem_num);
	int elem_n = end - beg;
	T* data = new T[elem_n];
	memcpy(data, _data + beg, sizeof(T) * elem_n);
	copyObj = SmartArray<T>(elem_n, data);
}

template <typename T>
void SmartArray<T>::copyIn(uint pos_destination, uint pos_source, uint elem_n,
	const SmartArray<T>& rth)
{
	if (_elem_num < pos_destination + elem_n) {
		resize(pos_destination + elem_n);
	}

	memcpy(_data + pos_destination, rth.buffer() + pos_source, sizeof(T) * elem_n);
	return;
}

template <typename T>
void SmartArray<T>::clear()
{
	RefManager::getInstance()->referenceReduce2<T>(_data);
	_data = NULL;
	_elem_num = 0;
}

template <typename T>
bool SmartArray<T>::empty() const
{
	return _elem_num == 0;
}

template <typename T>
void SmartArray<T>::resize(uint n)
{
	if (n == _elem_num) {
		return;
	}

	if (n == 0) {
		clear();
		return;
	}

	T* data = new T[n];
	assert(data);

	if (n > _elem_num) {
		memcpy(data, _data, sizeof(T) * _elem_num);
	}
	else {
		memcpy(data, _data, sizeof(T) * n);
	}

	clear();
	_elem_num = n;
	_data = data;
	RefManager::getInstance()->referenceAdd(_data);
}