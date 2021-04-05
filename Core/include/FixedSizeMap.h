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

#ifndef  MPCDPS_FIXEDSIZEMAP_H
#define  MPCDPS_FIXEDSIZEMAP_H

#include "SmartArray.h"

namespace mpcdps {

    /*A FixedSizeMap is map that has fixed size.*/
    template <typename T1, typename T2>
    class FixedSizeMap
    {
    public:
		typedef T1 KeyType;
		typedef T2 ValueType;

        /*Constructor: construct a FixedSizeMap of size n.*/
        FixedSizeMap(int n);

        /*Copy constructor.*/
        FixedSizeMap(const FixedSizeMap<T1, T2>& rth);

        /*Return size.*/
        int  size() const;

        /*The head of map.*/
        int head()  const;

        /*The tail of map.*/
        int tail()    const;

        /*Is empty.*/
        bool empty() const;

        /*Is full.*/
        bool full()  const;

        /*Insert a element (key, value).*/
        bool insert(T1 elem_key, const T2& elem_value);

        /*The key of tail. If tail() = -1, return -1.*/
        KeyType tailKey()   const;

        /*The key of head. If head() = -1, return -1.*/
        KeyType headKey() const;

        /*Gather all of the key and values.*/
        void elemList(std::vector<KeyType>& keys, std::vector<ValueType>& values) const;

        ValueType operator[](int i) const;

    private:
        /* Node in map.*/
        struct Node
        {
            T1  key;
            T2  value;
            int  pre;
            int  next;
        };

        SmartArray<Node> _data;  /* Data.*/
        int _head;  /* Head.*/
        int _tail;    /* Tail.*/
        int _next;   /* Next.*/
    };

#include "FixedSizeMap.inl"

}

#endif // MPCDPS_H
