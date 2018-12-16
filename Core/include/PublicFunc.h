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

#ifndef MPCDPS_PUBLICFUNC_H
#define MPCDPS_PUBLICFUNC_H

#include <vector>
#include "PublicInfo.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps
{
    /*
       The elements between [begPtId, endPtId) is the original set.
       filtereSet must be less than or equal order and each element must be unique.
       filtereSet must be a child set of the original set.
    */
    inline std::vector<int> filter(int begPtId, int endPtId, const std::vector<int>& filtereSet);

    /*
       vect and filtereSet must be less than or equal order and each element must be unique.
       filtereSet must be a child set of vect.
    */
    template <typename T>
    inline std::vector<T> filter(const std::vector<T>& vect, const std::vector<T>& filterSet);

    /*
       vect and excludeSet must be less than or equal order and each element must be unique.
       excludedPoints not must be a child set of vect
    */
    template <typename T>
    inline std::vector<T> exclude(const std::vector<T>& vect, const std::vector<T>& excludeSet);

    //The elements of vect1 and vect2 must be ordered as less than or equal order and each element must be unique.
    template <typename T>
    inline std::vector<T> merge(const std::vector<T>& vect1, const std::vector<T>& vect2);

    //The elements of vect1 and vect2 must be ordered as less than or equal order and each element must be unique.
    template <typename T>
    inline std::vector<T> intersect(const std::vector<T>& vect1, const std::vector<T>& vect2);

    //Sort for idList by keys, the keys and idList will change synchronously
    template <typename T>
    inline void sort_insert_syn(std::vector<T>& keys, std::vector<int>& idList);

    //Sort for idList by keys, the keys and idList will change synchronously
    template <typename T>
    inline void sort_shell_syn(std::vector<T>& keys, std::vector<int>& idList);

    //Sort for idList by keys, the keys and idList will change synchronously
    template <typename T>
    inline void sort_merge_syn(std::vector<T>& keys, std::vector<int>& idList);

    //Rearrange the keys as ascend order.
    template <typename T>
    inline void sort_shell(std::vector<T>& keys);

    //Rearrange the keys as ascend order.
    template <typename T>
    inline void sort_merge(std::vector<T>& keys);

    //Sort the idList by keys, the keys vector will not be changed.
    template <typename T>
    inline void sort_shell(std::vector<int>& idList, const std::vector<T>& keys);

    /*find the element in elemList, return id or -1.
       Note: elemList must be ordered ascending.
    */
    template <typename T>
    inline int find(const std::vector<T>& elemList, const T v);

    /*
       return i which e[i] <= v <= e[i+1]
       Note: elemList must be ordered ascending.
                return -1 if e[0]<v or e[n-1]>v;
    */
    template <typename T>
    inline int find_low_bound(const std::vector<T>& elemList, const T v);

    /*
       return relative position of each elements of vect_child in vect
       vect and vect_son must be less than or equal order and each element must be unique.
       vect_child must be a child set of vect.
    */
    template <typename T>
    inline std::vector<int> locate_relative(const std::vector<T>& vect, const std::vector<T>& vect_child);

    //vect must be ordered ascending.
    template <typename T>
    inline void filter_duplicate(std::vector<T>& vect);

    template <typename T>
    inline std::vector<T> make_vector(int n);

#include "PublicFunc.inl"

}

#endif