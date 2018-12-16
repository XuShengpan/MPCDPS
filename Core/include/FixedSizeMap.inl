/*
**********************************************************************
*
* This file is a part of library MPCDPS(Massive Point Cloud Data Processing System).
* It is a free program and it is protected by the license GPL-v3.0, you may not use the
* file except in compliance with the License.
*
* Copyright(c) 2016 - 2018 Xu Shengpan, all rights reserved.
*
* Email: jack_1227x@163.com
*
**********************************************************************
*/

template <typename T1, typename T2>
FixedSizeMap<T1, T2>::FixedSizeMap(int n)
	:_head(-1), _tail(-1), _next(0)
{
	_data.resize(n);
	for (int i = 0; i < n; ++i) {
		_data[i].next = i + 1;
		_data[i].pre = i - 1;
	}

	_tail = -1;
	_head = -1;
	_next = 0;
}

template <typename T1, typename T2>
FixedSizeMap<T1, T2>::FixedSizeMap(const FixedSizeMap<T1, T2>& rth)
{
	_data = rth._data;
	_head = rth._head;
	_tail = -1;
	_next = rth._next;
}

template <typename T1, typename T2>
int   FixedSizeMap<T1, T2>::size() const
{
	return _data.size();
}

template <typename T1, typename T2>
int FixedSizeMap<T1, T2>::head()  const
{
	return _head;
}

template <typename T1, typename T2>
int FixedSizeMap<T1, T2>::tail()  const
{
	return _tail;
}

template <typename T1, typename T2>
bool FixedSizeMap<T1, T2>::empty() const
{
	return _head == -1;
}

template <typename T1, typename T2>
bool FixedSizeMap<T1, T2>::full() const
{
	return _next == size();
}

template <typename T1, typename T2>
bool FixedSizeMap<T1, T2>::insert(T1 ek, const T2& ev)
{
	if (full()) {
		if (_data[_tail].key <= ek) {
			return false;
		}
		int k = _data[_tail].pre;
		_data[k].next = -1;
		_data[_tail].pre = -1;
		_data[_tail].next = _next;
		_next = _tail;
		_tail = k;
	}
	int k = _next;
	_next = _data[_next].next;
	_data[k].key = ek;
	_data[k].value = ev;

	int i = _tail;
	while (i != -1 && _data[i].key >= ek) {
		i = _data[i].pre;
	}
	if (i == -1) {
		if (_head != -1) {
			_data[k].pre = -1;
			_data[k].next = _head;
			_data[_head].pre = k;
			_head = k;
		}
		else {
			_data[k].pre = -1;
			_data[k].next = -1;
			_head = k;
			_tail = k;
		}
	}
	else {
		int j = _data[i].next;
		_data[i].next = k;
		_data[k].pre = i;
		_data[k].next = j;
		if (j == -1) {
			_tail = k;
		}
		else {
			_data[j].pre = k;
		}
	}
	return true;
}

template <typename T1, typename T2>
T1 FixedSizeMap<T1, T2>::tailKey() const
{
	return _tail == -1 ? -1 : _data[_tail].key;
}

template <typename T1, typename T2>
T1 FixedSizeMap<T1, T2>::headKey() const
{
	return _head == -1 ? -1 : _data[_head].key;
}

template <typename T1, typename T2>
void FixedSizeMap<T1, T2>::elemList(
	std::vector < KeyType>& keys,
	std::vector < ValueType>& values) const
{
	int i = head();
	while (i != -1) {
		keys.push_back(_data[i].key);
		values.push_back(_data[i].value);
		i = _data[i].next;
	}
}