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

#ifndef  MPCDPS_PUBLICFUNC_H
#define MPCDPS_PUBLICFUNC_H

#include <vector>
#include "PublicInfo.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

/*
   The elements between [beg, end) is the original set.
   filteredSet must be less than or equal order and each element must be unique.
   filteredSet must be a child set of the original set.
*/
inline std::vector<int> filter(int beg, int end, const std::vector<int>& filteredSet)
{
    int i = beg, j = 0, k = 0;
    int n = filteredSet.size();
    std::vector<int> vec(end - beg - n);
    while (i < end && j < n) {
        if (i < filteredSet[j]) {
            vec[k++] = i;
            ++i;
        } else {
            assert(filteredSet[j] == i);
            ++j;
            ++i;
        }
    }

    for (; i < end; ++i)
        vec[k++] = i;
    return vec;
}
/*
   vect and filteredSet must be less than or equal order and each element must be unique.
   filteredSet must be a child set of vect.
*/
template <typename T>
inline std::vector<T> filter(const std::vector<T>& vect, const std::vector<T>& filteredSet)
{
    if (filteredSet.empty())
        return vect;

    int n = vect.size(), n1 = filteredSet.size();
    std::vector<T> res(n - n1);
    int i = 0, j = 0, k = 0;
    while (i < n) {
        if (vect[i] < filteredSet[j]) {
            res[k++] = vect[i];
            ++i;
        } else {
            assert(vect[i] == filteredSet[j]);
            ++j;
            ++i;
            if (j == n1)
                break;
        }
    }

    for (; i < n; ++i)
        res[k++] = vect[i];
    assert(k == n - n1);
    return res;
}

/*
   vect and excludedSet must be less than or equal order and each element must be unique.
   excludedSet not must be a child set of vect
*/
template <typename T>
inline std::vector<T> exclude(const std::vector<T>& vect, const std::vector<T>& excludedSet)
{
    if (excludedSet.empty())
        return vect;
    if (vect.empty())
        return vect;

    std::vector<T> newList(vect.size());

    int i = 0, j = 0, k = 0;
    int n1 = vect.size(), n2 = excludedSet.size();
    while (i < n1) {
        if (j < n2) {
            while (i < n1) {
                if (vect[i] < excludedSet[j])
                    newList[k++] = vect[i++];
                else if (vect[i] == excludedSet[j]) {
                    ++i;
                    break;
                } else
                    break;
            }

            if (i == n1) break;
            while (j < n2 && excludedSet[j] < vect[i]) ++j;

        } else {
            while (i < n1) newList[k++] = vect[i++];
        }
    }

    newList.resize(k);
    return newList;
}

/*
Merge the two vector and return the result.

The elements of vect1 and vect2 must be ordered as less than or equal order 
and each element must be unique.
*/
template <typename T>
inline std::vector<T> merge(const std::vector<T>& vect1, const std::vector<T>& vect2)
{
    if (vect2.empty())
        return vect1;
    if (vect1.empty())
        return vect2;

    int i = 0, j = 0, k = 0;
    int n1 = vect1.size(), n2 = vect2.size();
    std::vector<T> newList(n1 + n2);

    while (i < n1 || j < n2) {
        if (j < n2) {
            while (i < n1 && vect1[i] < vect2[j])  newList[k++] = vect1[i++];

            if (i < n1) {
                if (vect1[i] == vect2[j]) {
                    newList[k++] = vect1[i++];
                    ++j;
                } else {
                    while (j < n2 && vect2[j] < vect1[i])
                        newList[k++] = vect2[j++];
                }
            } else {
                while (j < n2)  newList[k++] = vect2[j++];
            }
        } else {
            while (i < n1)  newList[k++] = vect1[i++];
        }
    }

    newList.resize(k);
    return newList;

}

/*
Get the intersection of the two vectors.

The elements of vect1 and vect2 must be ordered as less than or equal order and each element must be unique.
*/
template <typename T>
inline std::vector<T> intersect(const std::vector<T>& vect1, const std::vector<T>& vect2)
{
    int i = 0, j = 0, k = 0, n1 = vect1.size(), n2 = vect2.size();
    std::vector<T> vect(MINV(n1, n2));

    while (i < n1 && j < n2) {
        while (i < n1 && vect1[i] < vect2[j]) ++i;
        if (i == n1)
            break;
        else if (vect1[i] > vect2[j]) {
            while (j < n2 && vect2[j] < vect1[i]) ++j;
            if (j == n2)
                break;
            else if (vect2[j] == vect1[i]) {
                vect[k++] = vect1[i];
                ++i;
                ++j;
            }
        } else {
            vect[k++] = vect1[i];
            ++i;
            ++j;
        }
    }

    vect.resize(k);
    return vect;
}

/*
Sort for idList by keys, the keys and idList will change synchronously
*/
template <typename T>
inline void sort_insert_syn(std::vector<T>& keys, std::vector<int>& idList)
{
    int i, j;
    T theKey;
    int id;

    for (i = 1; i < idList.size(); ++i) {
        theKey = keys[i];
        id = idList[i];
        j = i - 1;
        while (j >= 0 && keys[j] > theKey) {
            keys[j + 1] = keys[j];
            idList[j + 1] = idList[j];
            --j;
        }
        keys[j + 1] = theKey;
        idList[j + 1] = id;
    }
}

/*
Sort for idList by keys, the keys and idList will change synchronously
*/
template <typename T>
inline void sort_shell_syn(std::vector<T>& keys, std::vector<int>& idList)
{
    int i, j, n = idList.size();
    T theKey;
    int id;

    int d = idList.size() / 2;
    while (d > 0) {
        for (i = d; i < n; ++i) {
            theKey = keys[i];
            id = idList[i];
            j = i - d;
            while (j >= 0 && keys[j] > theKey) {
                keys[j + d] = keys[j];
                idList[j + d] = idList[j];
                j -= d;
            }

            keys[j + d] = theKey;
            idList[j + d] = id;
        }

        d = d >> 1;
    }
}

//[low, high)
template <typename T>
inline void merge2(std::vector<T>& keys, std::vector<int>& idList, uint low, uint mid, uint high)
{
    uint i, j, k, n1 = mid - low;
    std::vector<int> idList1(n1);
    std::vector<T> keys1(n1);
    for (i = 0; i < n1; ++i) {
        keys1[i] = keys[low + i];
        idList1[i] = idList[low + i];
    }

    i = 0;
    j = mid;
    k = low;
    while (i < n1 && j < high) {
        if (keys1[i] <= keys[j]) {
            keys[k] = keys1[i];
            idList[k] = idList1[i];
            ++k;
            ++i;
        } else {
            keys[k] = keys[j];
            idList[k] = idList[j];
            ++k;
            ++j;
        }
    }

    while (i < n1) {
        keys[k] = keys1[i];
        idList[k] = idList1[i];
        ++k;
        ++i;
    }
}

/*
Sort for idList by keys, the keys and idList will change synchronously
*/
template <typename T>
inline void sort_merge_syn(std::vector<T>& keys, std::vector<int>& idList)
{
    uint i, d, n = idList.size();
    for (d = 1; d < n; d = d << 1) {
        for (i = 0; i + 2 * d - 1 < n; i = i + 2 * d)
            merge2(keys, idList, i, i + d, i + 2 * d);
        if (i + d < n)
            merge2(keys, idList, i, i + d, n);
    }
}

//Rearrange the keys as ascending order.
template <typename T>
inline void sort_shell(std::vector<T>& keys)
{
    int i, j, n = keys.size();
    T theKey;

    for (int d = n / 2; d > 0; d >>= 1) {
        for (i = d; i < n; ++i) {
            theKey = keys[i];
            j = i - d;
            while (j >= 0 && keys[j] > theKey) {
                keys[j + d] = keys[j];
                j -= d;
            }

            keys[j + d] = theKey;
        }
    }
}

//[low, high)
template <typename T>
inline void merge2(std::vector<T>& keys, uint low, uint mid, uint high)
{
    uint i, j, k, n1 = mid - low;
    std::vector<T> keys1(n1);
    for (i = 0; i < n1; ++i) {
        keys1[i] = keys[low + i];
    }

    i = 0;
    j = mid;
    k = low;
    while (i < n1 && j < high) {
        if (keys1[i] <= keys[j]) {
            keys[k] = keys1[i];
            ++k;
            ++i;
        } else {
            keys[k] = keys[j];
            ++k;
            ++j;
        }
    }

    while (i < n1) {
        keys[k] = keys1[i];
        ++k;
        ++i;
    }
}

//Rearrange the keys as ascending order.
template <typename T>
inline void sort_merge(std::vector<T>& keys)
{
    uint i, d, n = keys.size();
    for (d = 1; d < n; d = d << 1) {
        for (i = 0; i + 2 * d - 1 < n; i = i + 2 * d)
            merge2(keys, i, i + d, i + 2 * d);
        if (i + d < n)
            merge2(keys, i, i + d, n);
    }
}

//Sort the idList by keys, the keys vector will not be changed.
template <typename T>
inline void sort_shell(std::vector<int>& idList, const std::vector<T>& keys)
{
    int i, j, n = idList.size();
    T theKey;
    int d = idList.size() / 2;
    uint id;
    while (d > 0) {
        for (i = d; i < n; ++i) {
            id = idList[i];
            theKey = keys[id];
            j = i - d;
            while (j >= 0 && keys[idList[j]] > theKey) {
                idList[j + d] = idList[j];
                j -= d;
            }

            idList[j + d] = id;
        }

        d = d >> 1;
    }
}

/*find the element in elemList, return id or -1.
   Note: elemList must be ordered ascending.
*/
template <typename T>
inline int find(const std::vector<T>& elemList, const T v)
{
    int i = 0, j = elemList.size() - 1, k;
    if (v<elemList[i] || v>elemList[j]) return -1;
    while (i <= j) {
        k = (i + j) / 2;
        if (elemList[k] == v)
            return k;
        else if (elemList[k] > v)
            j = k - 1;
        else
            i = k + 1;
    }
    return -1;
}

/*
    return relative position of each elements of vect_child in vect
    vect and vect_son must be less than or equal order and each element must be unique.
    vect_child must be a child set of vect.
*/
template <typename T>
inline std::vector<int> locate_relative(const std::vector<T>& vect, const std::vector<T>& vect_child)
{
    std::vector<int> vect1;
    int n = vect.size(), n1 = vect_child.size(), k = 0;
    for (int i = 0; i < n; ++i) {
        if (vect[i] == vect_child[k]) {
            vect1.push_back(i);
            ++k;
            if (k == n1)
                break;
        }
    }
    return vect1;
}

/*
filter the duplicate elements in vect.
vect must be ordered ascending.
*/
template <typename T>
inline void filter_duplicate(std::vector<T>& vect)
{
    double t0 = -DBL_MAX;
    int k = 0, n = vect.size();
    for (int i = 0; i < n; ++i) {
        if (vect[i] > t0) {
            vect[k++] = t0 = vect[i];
        }
    }
    vect.resize(k);
}

//make a vector of [0, 1, ..., n-1]
template <typename T>
inline std::vector<T> make_vector(int n)
{
    std::vector<T> vect(n);
    for (int i = 0; i < n; ++i)
        vect[i] = i;
    return std::move(vect);
}

/*
       return i which e[i] <= v < e[i+1]
       Note: elemList must be ordered ascending.
                return -1 if e[0]<v or e[n-1]>v;
*/
template <typename T>
inline int find_low_bound(const std::vector<T>& elemList, const T v)
{
    int n = elemList.size();
    if (v < elemList[0] || v > elemList[n - 1])
        return -1;

    int li = 0, ri = n - 1, i;
    if (v == elemList[li])
        return li;
    if (v == elemList[ri])
        return ri;

    while (ri - li > 1) {
        i = (li + ri) / 2;
        if (v == elemList[i]) {
            return i;
        } if (v < elemList[i])
            ri = i;
        else
            li = i;
    }
    return li;
}

}

#endif