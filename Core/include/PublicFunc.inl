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

std::vector<int> filter(int begPtId, int endPtId, const std::vector<int>& filteredPoints)
{
    int i = begPtId, j = 0, k = 0;
    int n = filteredPoints.size();
    std::vector<int> vec(endPtId - begPtId - n);
    while (i < endPtId && j < n) {
        if (i < filteredPoints[j]) {
            vec[k++] = i;
            ++i;
        } else {
            assert(filteredPoints[j] == i);
            ++j;
            ++i;
        }
    }

    for (; i < endPtId; ++i)
        vec[k++] = i;
    return vec;
}

template <typename T>
std::vector<T> filter(const std::vector<T>& vect, const std::vector<T>& filteredPoints)
{
    if (filteredPoints.empty())
        return vect;

    int n = vect.size(), n1 = filteredPoints.size();
    std::vector<T> res(n - n1);
    int i = 0, j = 0, k = 0;
    while (i < n) {
        if (vect[i] < filteredPoints[j]) {
            res[k++] = vect[i];
            ++i;
        } else {
            assert(vect[i] == filteredPoints[j]);
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

template <typename T>
std::vector<T> exclude(const std::vector<T>& vect, const std::vector<T>& excludedPoints)
{
    if (excludedPoints.empty())
        return vect;
    if (vect.empty())
        return vect;

    std::vector<T> newList(vect.size());

    int i = 0, j = 0, k = 0;
    int n1 = vect.size(), n2 = excludedPoints.size();
    while (i < n1) {
        if (j < n2) {
            while (i < n1) {
                if (vect[i] < excludedPoints[j])
                    newList[k++] = vect[i++];
                else if (vect[i] == excludedPoints[j]) {
                    ++i;
                    break;
                } else
                    break;
            }

            if (i == n1) break;
            while (j < n2 && excludedPoints[j] < vect[i]) ++j;

        } else {
            while (i < n1) newList[k++] = vect[i++];
        }
    }

    newList.resize(k);
    return newList;
}

template <typename T>
std::vector<T> merge(const std::vector<T>& vect1, const std::vector<T>& vect2)
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

template <typename T>
std::vector<T> intersect(const std::vector<T>& vect1, const std::vector<T>& vect2)
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

template <typename T>
void sort_insert_syn(std::vector<T>& keys, std::vector<int>& idList)
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

template <typename T>
void sort_shell_syn(std::vector<T>& keys, std::vector<int>& idList)
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
void merge2(std::vector<T>& keys, std::vector<int>& idList, uint low, uint mid, uint high)
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

template <typename T>
void sort_merge_syn(std::vector<T>& keys, std::vector<int>& idList)
{
    uint i, d, n = idList.size();
    for (d = 1; d < n; d = d << 1) {
        for (i = 0; i + 2 * d - 1 < n; i = i + 2 * d)
            merge2(keys, idList, i, i + d, i + 2 * d);
        if (i + d < n)
            merge2(keys, idList, i, i + d, n);
    }
}

template <typename T>
void sort_shell(std::vector<T>& keys)
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
void merge2(std::vector<T>& keys, uint low, uint mid, uint high)
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

template <typename T>
void sort_merge(std::vector<T>& keys)
{
    uint i, d, n = keys.size();
    for (d = 1; d < n; d = d << 1) {
        for (i = 0; i + 2 * d - 1 < n; i = i + 2 * d)
            merge2(keys, i, i + d, i + 2 * d);
        if (i + d < n)
            merge2(keys, i, i + d, n);
    }
}

template <typename T>
void sort_shell(std::vector<int>& idList, const std::vector<T>& keys)
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

template <typename T>
int find(const std::vector<T>& elemList, const T v)
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

template <typename T>
std::vector<int> locate_relative(const std::vector<T>& vect, const std::vector<T>& vect_child)
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

template <typename T>
void filter_duplicate(std::vector<T>& vect)
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

template <typename T>
std::vector<T> make_vector(int n)
{
    std::vector<T> vect(n);
    for (int i = 0; i < n; ++i)
        vect[i] = i;
    return std::move(vect);
}

template <typename T>
int find_low_bound(const std::vector<T>& elemList, const T v)
{
    int n = elemList.size();
    if (v < elemList[0] || v > elemList[n - 1])
        return -1;

    int li = 0, ri = n - 1, i;
    while (ri - li > 1) {
        i = (li + ri) / 2;
        if (v == elemList[i]) {
            if (i == n - 1) {
                li = i - 1;
                ri = i;
            } else {
                ri = i;
                li = i + 1;
            }
        } if (v < elemList[i])
            ri = i;
        else
            li = i;
    }
    return li;
}