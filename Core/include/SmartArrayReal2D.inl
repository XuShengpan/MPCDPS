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

template <typename T>
SmartArrayReal2D<T>::SmartArrayReal2D(void) :_data(NULL), _cn(0), _rn(0)
{

}

template <typename T>
SmartArrayReal2D<T>::~SmartArrayReal2D(void)
{
	clear();
}

template <typename T>
SmartArrayReal2D<T>::SmartArrayReal2D(uint rn, uint cn, const T* data)
	:_cn(cn), _rn(rn), _data(NULL)
{
	int len = rn * cn;
	if (len > 0) {
		_data = new T[len];
		if (data) {
			memcpy(_data, data, len * sizeof(T));
		}
		RefManager::getInstance()->referenceAdd(_data);
	}
}

template <typename T>
SmartArrayReal2D<T> SmartArrayReal2D<T>::clone() const
{
	SmartArrayReal2D<T> obj(_rn, _cn);
	memcpy(obj._data, _data, sizeof(T) * _rn * _cn);
	return obj;
}

template <typename T>
SmartArrayReal2D<T>::SmartArrayReal2D(const SmartArrayReal2D<T>& rth)
{
	_cn = rth.colCount();
	_rn = rth.rowCount();
	_data = rth.buffer();
	if (_data) {
		RefManager::getInstance()->referenceAdd(_data);
	}
}

template <typename T>
SmartArrayReal2D<T> SmartArrayReal2D<T>::block(uint r0, uint c0, uint rn, uint cn)  const
{
	SmartArrayReal2D<T> blk(rn, cn);
	for (uint ri = 0; ri < rn; ++ri) {
		for (uint ci = 0; ci < cn; ++ci) {
			blk[ri][ci] = _data[linearId(r0 + ri, c0 + ci)];
		}
	}
	return blk;
}

template <typename T>
void SmartArrayReal2D<T>::setBlock(const SmartArrayReal2D<T>& blk, 
	uint r0_dest, uint c0_dest, 
	int rn /* = -1 */, int cn /* = -1 */, 
	uint r0_sourc /* = 0 */, uint c0_sourc /* = 0 */)
{
	if (rn == -1) {
		rn = blk.rowCount();
	}
	if (cn == -1) {
		cn = blk.colCount();
	}

	assert(r0_dest + rn <= _rn && c0_dest + cn <= _cn &&
		r0_sourc + rn <= blk.rowCount() && c0_sourc + cn <= blk.colCount());

	for (uint r = 0; r < rn; ++r) {
		for (uint c = 0; c < cn; ++c) {
			_data[linearId(r0_dest + r, c0_dest + c)] = blk[r0_sourc + r][c0_sourc + c];
		}
	}
}

template <typename T>
void SmartArrayReal2D<T>::clear()
{
	if (_data) {
		RefManager::getInstance()->referenceReduce2<T>(_data);
		_data = NULL;
		_cn = 0;
		_rn = 0;
	}
}

template <typename T>
void SmartArrayReal2D<T>::resize(uint rn, uint cn)
{
	if (cn == _cn && rn == _rn) {
		return;
	}

	if (cn == 0 || rn == 0) {
		clear();
		return;
	}

	T* data = new T[rn * cn];

	int rn1 = MINV(rn, _rn), cn1 = MINV(cn, _cn);
	for (int r = 0; r < rn1; ++r) {
		for (int c = 0; c < cn1; ++c) {
			data[r * cn + c] = _data[r * _cn + c];
		}
	}

	clear();

	_data = data;
	_cn = cn;
	_rn = rn;
	RefManager::getInstance()->referenceAdd(_data);
}

template <typename T>
void SmartArrayReal2D<T>::reset(T t)
{
	int n = _rn * _cn;
	for (int i = 0; i < n; ++i) {
		_data[i] = t;
	}
}

template <typename T>
void SmartArrayReal2D<T>::operator = (const SmartArrayReal2D<T>& rth)
{
	if (_data == rth.buffer()) return;
	clear();
	_cn = rth.colCount();
	_rn = rth.rowCount();
	_data = rth.buffer();
	if (_data) {
		RefManager::getInstance()->referenceAdd(_data);
	}
}

template <typename T>
T* SmartArrayReal2D<T>::operator[] (const uint i) const
{
	assert(i < _rn);
	return _data + (i * _cn);
}

template <typename T>
T SmartArrayReal2D<T>::operator () (uint r, uint c) const
{
	return _data[r * _cn + c];
}

template <typename T>
T& SmartArrayReal2D<T>::operator () (uint r, uint c)
{
	return _data[r * _cn + c];
}

template <typename T>
T*  SmartArrayReal2D<T>::buffer() const
{
	return _data;
}

template <typename T>
uint SmartArrayReal2D<T>::colCount() const
{
	return _cn;
}

template <typename T>
uint SmartArrayReal2D<T>::rowCount() const
{
	return _rn;
}

template <typename T>
bool SmartArrayReal2D<T>::empty() const
{
	return (_rn == 0);
}