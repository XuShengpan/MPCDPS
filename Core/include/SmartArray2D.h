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

#ifndef MPCDPS_SMARTARRAY2D_H
#define MPCDPS_SMARTARRAY2D_H

#include "RefManager.h"

namespace mpcdps {

/*A SmartArray2D works just like a SmartArray, but the element of a SmartArray2D is a list of some objects
 * which has the fixed size of WIDTH.
 */
template <typename T, uint WIDTH>
class SmartArray2D
{
public:
    /* Default Constructor */
    SmartArray2D(void);

    /* Constructor: construct a SmartArray2D object with an existing pointer of array which has size of length. */
    SmartArray2D(uint length, T* data = NULL);

    /* Copy Constructor.*/
    SmartArray2D(const SmartArray2D< T, WIDTH>& rth);

    /* Destructor.*/
    ~SmartArray2D(void);

    typedef T  DataType;   /* typedef DataType as T  */
    typedef T* ElemType;  /* typedef ElemType as T* */

    /* Clone this object.*/
    SmartArray2D  clone() const;

    /* Initialize this object with the existing object rth.*/
    void operator = (const SmartArray2D< T, WIDTH>& rth);

    /*Append the other object to end of the object, returns the beginning position of the
     * data of other object in the result object.
     * If rth is empty, it returns -1.
     */
    inline int  append(const SmartArray2D< T, WIDTH>& rth);

    /*Copy out elements of range [beg, end)  */
    inline void copyOut(uint beg, uint end, SmartArray2D< T, WIDTH>& copyObj);

    /*Copy the data in other object to this object, pos specified the begin position,
     * num means the elem number to be copied.

     * If the size of this object is less than pos + rth.size(), this object will be resized.
     * If the copy position of this object has data already, the data will be overwrite.
     */
    inline void copyIn(uint pos_dest, uint pos_src, uint elem_n, const SmartArray2D< T, WIDTH>& rth);

    /* Clear the object. */
    inline void clear();

    /* Resize the object to size n.*/
    inline void resize(uint n);

    /* [] operator */
    inline T* operator[] (const uint i) const;

    /* To get the i'th element's address */
    inline T* ptrElem(uint i) const;

    /* To get the size of the SmartArray2D */
    inline uint size() const;

    /* Return the buffer.*/
    inline T*  buffer() const;

    /* Return is empty*/
    inline bool empty() const;

protected:
    T*   _data;   /* Data buffer*/
    uint _elem_num;  /* Size of elements*/
};

#include "SmartArray2D.inl"

}

#endif