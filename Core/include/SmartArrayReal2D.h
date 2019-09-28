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

#ifndef  MPCDPS_SMARTARRAYREAL2D_H
#define MPCDPS_SMARTARRAYREAL2D_H

#include "RefManager.h"

namespace mpcdps {

/*A SmartArrayReal2D is a matrix whose elements can be not necessarily to be a number.
*   We can define a SmartArrayReal2D and access its elements just like visit a matrix.   
*   e.g. We can define a 1000 x 2000 pixmap like this:
*       
*       //definition for Rgba;
*       SmartArrayReal2D<Rgba> pixm(1000, 200);
*       //visit its (100, 101) pixel:
*       Rgba& p = pixm[100][101];
*/
template <typename T>
class SmartArrayReal2D
{
public:
    /* Default constructor.*/
    SmartArrayReal2D(void);

    /* Construct a SmartArrayReal2D whose size is rn rows and cn columns
     * and initialize with data, if data is NULL, the SmartArrayReal2D will initialize 
     * with default constructor of T.
     */
    SmartArrayReal2D(uint rn, uint cn, const T* data = NULL);

    /* Copy constructor.*/
    SmartArrayReal2D(const SmartArrayReal2D<T>& rth);

    /* Destructor.*/
    ~SmartArrayReal2D(void);

    /* Clone this object to another object.*/
    SmartArrayReal2D<T> clone() const;

    typedef T  DataType;

    /* = operator */
    inline void operator = (const SmartArrayReal2D<T>& rth);

    /* Copy out a block from this object at position (r0, c0) with size (rn, cn),
         Return a empty block object in case of failed.
    */
    inline SmartArrayReal2D<T> block(uint r0, uint c0, uint rn, uint cn)  const;

    /*Copy into this object from a block. If failed, it will do nothing.
	   if cn = -1, cn = blk.column();
	   if rn = -1, rn = blk.row();
	*/
    inline void setBlock(const SmartArrayReal2D<T>& blk, uint r0_dest, uint c0_dest, int rn = -1, int cn = -1, uint r0_sourc = 0, uint c0_sourc = 0);

    /*  Clear the object.*/
    inline void clear();

    /*  Resize the object to rn x cn.*/
    inline void resize(uint rn, uint cn);

    /*  Reset all of the Matrix element to t.*/
    inline void reset(T t);

    inline T* operator[] (const uint i) const;
    inline T operator () (uint r, uint c) const;
    inline T& operator () (uint r, uint c);

    /* Get the pointer of the buffer.*/
    inline T*   buffer() const;

    /*  Get column count.*/
    inline uint colCount()  const;

    /*  Get row count.*/
    inline uint rowCount() const;

    /* Test whether it is empty.*/
    inline bool empty()  const;

protected:
    inline int linearId(uint r, uint c)  const {return r*_cn + c;}

protected:
    T*   _data;   /* Data buffer.*/
    uint _rn;  /* Row count. */
    uint _cn;  /* Column count.*/
};

#include "SmartArrayReal2D.inl"

}

#endif
