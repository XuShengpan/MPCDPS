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

#ifndef  MPCDPS_SMARTARRAY_H
#define MPCDPS_SMARTARRAY_H

#include "RefManager.h"

namespace mpcdps {

/*A smart array is a array which has interface size() and can be visit just lick normal arrays.
 *  You don't need to care about the memory management, the smart array maintain it by itself.
 *  It is more convenient than normal array. 
 */
template <typename T>
class SmartArray
{
public:
    /*Default constructor.*/
    SmartArray(void);

    /* Construct a smart array with the exist buffer which has size elem_n. */
    SmartArray(uint elem_n, T* data = NULL);

    /* Copy constructor.*/
    SmartArray(const SmartArray<T>& rth);

    /* Clone the object.*/
    SmartArray clone() const;

    /* Destructor.*/
    virtual ~SmartArray(void);

    /*Append the other object to end of the object, returns the beginning position of the 
     * data of other object in the result object.
     * If rth is empty, it returns -1.
     */
    inline int append(const SmartArray<T>& rth); 

    /* Element pointer of i'th element.*/
    inline T* ptrElem(const uint i) const;

    /* = operator the class. */
    inline void operator = (const SmartArray<T>& rth);

    /*Copy out elements[beg, end)  */
    inline void copyOut(uint beg, uint end, SmartArray<T>& copyObj);

    /*Copy the data in other object to this object, pos specified the begin position, 
     * num means the elem number to be copied.
      
     * If the size of this object is less than pos + rth.size(), this object will be resized.
     * If the copy position of this object has data already, the data will be overwrite.
    */
    inline void copyIn(uint pos_destination, uint pos_source, uint elem_n, const SmartArray<T>& rth);

    /* Clear the object. */
    inline void clear();

    /* Resize the object to size n.*/
    inline void resize(uint n);

    /*Reset all of the element value to t. */
    inline void reset(const T& t);

    inline T& operator[] (const uint i);
    inline const T& operator[] (const uint i) const;

    /*Return the size of the array.  */
    inline uint size() const;

    /* Return the buffer.*/
    inline T* buffer() const;

    /* Return whether is empty. */
    inline bool empty() const;

protected:
    uint  _elem_num;
    T* _data;
};

#include "SmartArray.inl"

}

#endif
