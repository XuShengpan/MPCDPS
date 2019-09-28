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

#ifndef  MPCDPS_SMARTPOINTER_H
#define MPCDPS_SMARTPOINTER_H

#include "RefManager.h"

namespace mpcdps{

template <typename T>
class SmartPointer
{
public:
	SmartPointer(T* data = NULL);
	SmartPointer(const SmartPointer<T>& otherObj);

	virtual ~SmartPointer(void);

    inline operator T* () const;
    inline T& operator* () const;
    inline T* operator-> () const;

    inline T* operator= (T* data);
    inline SmartPointer<T>& operator= (const SmartPointer<T>& otherObj);

    inline bool operator== (T* data) const;
    inline bool operator!= (T* data) const;

    inline T* get() const;
    inline void clear();

protected:
    T* mData;

};

#include "SmartPointer.inl"

}

#endif
