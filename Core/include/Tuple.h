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

#ifndef   MPCDPS_TUPLE_H
#define  MPCDPS_TUPLE_H

#include <ostream>
#include "PublicInfo.h"

namespace mpcdps {

	/* A tuple is a fixed size array.*/
    template <typename T, int K>
    class Tuple
    {
    public:

        /* Default constructor.*/
        Tuple()
        {
            for (int i = 0; i < K; ++i)
                _data[i] = T();
        }

        /* Construct a object with data.*/
        Tuple(const T* data)
        {
            for (int i = 0; i < K; ++i)
                _data[i] = data[i];
        }

        /* Copy constructor.*/
        Tuple(const Tuple<T, K>& obj)
        {
            for (int i = 0; i < K; ++i)
                _data[i] = obj._data[i];
        }

        /* Destructor.*/
        ~Tuple()
        {

        }

        /* const [] operator to access the i'th item.*/
        inline T operator[] (int i) const
        {
            return _data[i];
        }

        /* [] operator to access the reference of the i'th item.*/
        inline T& operator[](int i)
        {
            return _data[i];
        }

        /* To access the const buffer. */
        inline const T* const buffer() const
        {
            return _data;
        }

        /* To access the buffer. */
        inline T* const buffer()
        {
            return _data;
        }

        /* = operator. */
        inline void operator = (const Tuple<T, K>& obj)
        {
            for (int i = 0; i < K; ++i) {
                _data[i] = obj._data[i];
            }
        }

        inline void reset(T t)
        {
            for (int i = 0; i < K; ++i) {
                _data[i] = t;
            }
        }

    protected:
        T _data[K];   /* The data buffer.*/
    };

    template<typename T, int K>
    std::ostream& operator << (std::ostream& out, const Tuple<T, K>& tuple)
    {
        for (int i = 0; i < K; ++i) {
            out << tuple[i] << " ";
        }
        return out;
    }
}


#endif