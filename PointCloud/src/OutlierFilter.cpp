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

#include "OutlierFilter.h"
#include <algorithm>
#include "PublicFunc.h"
#include <stack>

namespace mpcdps {

template<typename T>
OutlierFilter<T>::OutlierFilter(void)
{
}

template<typename T>
OutlierFilter<T>::~OutlierFilter(void)
{
}

template<typename T>
void OutlierFilter<T>::initialize(const SmartArray2D<T, 3>& vtxAry, const Box3<T>& box, float distance_threshold)
{
    mVtxAry = vtxAry;
    mBox = box;
    mDistanceShreshold = distance_threshold;
    mGrid.initialize1(box.min(0), box.min(1), box.length(0), box.length(1), distance_threshold, distance_threshold);
    mGrid.initialize_z(box.min(2), box.length(2), distance_threshold);
}

template<typename T>
void OutlierFilter<T>::run()
{
	std::vector<int> ptids = make_vector<int>(mVtxAry.size());
	run(ptids);
}

template<typename T>
void OutlierFilter<T>::run(const std::vector<int>& ptIds)
{
	mOutilerPoints.clear();

    int r = 0;
    int c = 0;
    int h = 0;
    for (int i = 0; i < ptIds.size(); ++i) {
        auto vtx = mVtxAry[ptIds[i]];
        r = mGrid.get_r(vtx[1]);
        c = mGrid.get_c(vtx[0]);
        h = mGrid.get_h(vtx[2]);
        mGrid[r][c][h].push_back(ptIds[i]);
    }

    _rn = mGrid.rowCount();
    _cn = mGrid.colCount();
    _hn = mGrid.heightCount();
    doWork();
}

template<typename T>
std::vector<int> OutlierFilter<T>::getOutlierPoints() const
{
	return mOutilerPoints;
}

struct Index {
    int r = 0;
    int c = 0;
    int h = 0;

    Index(int r1, int c1, int h1)
    :r(r1), c(c1), h(h1) {

    }

    Index():r(0), c(0), h(0) {

    }
};

template<typename T>
void OutlierFilter<T>::doWork()
{
	_rn = mGrid.rowCount();
	_cn = mGrid.colCount();
	_hn = mGrid.heightCount();
    const int n = _rn * _cn * _hn;

    std::vector<Index> outlier_cells;

    std::vector<int> labels(n, 0);
    int next_label = 1;
    for (int r = 0; r < _rn; ++r) {
        for (int c = 0; c < _cn; ++c) {
            for (int h = 0; h < _hn; ++h) {
                int i = getLinearId(r, c, h);
                if (labels[i]) continue;

                int label = next_label++;
                std::stack<Index> stk;
                std::vector<Index> cells;

                stk.push(Index(r, c, h));
                labels[i] = label;
                cells.push_back(Index(r, c, h));

                while (!stk.empty()) {
                    Index idx = stk.top();
                    stk.pop();

                    for (int r1 = idx.r - 1; r1 <= idx.r + 1; ++r1) {
                        if (r1 < 0) continue;
                        if (r1 >= _rn) break;

                        for (int c1 = idx.c - 1; c1 <= idx.c + 1; ++c1) {
                            if (c1 < 0) continue;
                            if (c1 >= _cn) break;

                            for (int h1 = idx.h - 1; h1 <= idx.h + 1; ++h1) {
                                if (h1 < 0) continue;
                                if (h1 >= _hn) break;

                                if (!mGrid.is_empty(r1, c1, h1)) {
                                    int i1 = getLinearId(r1, c1, h1);
                                    if (labels[i1] == 0) {
                                        stk.push(Index(r1, c1, h1));
                                        labels[i1] = label;
                                        cells.push_back(Index(r1, c1, h1));
                                    }
                                }
                            }
                        }
                    }
                }

                bool tag = false;
                if (cells.size() <= 2) {
                    tag = true;
                } else if (cells.size() < 4){
                    int n_points = 0;
                    for (auto idx : cells) {
                        n_points += mGrid[idx.r][idx.c][idx.h].size();
                    }
                    if (n_points < mNumShreshold) {
                        tag = true;
                    }
                }

                if (tag) {
                    for (auto idx : cells) {
                        outlier_cells.push_back(idx);
                    }
                }
            }
        }
    }
    
    mOutilerPoints.clear();
    for (auto idx : outlier_cells) {
        const auto& ptids = mGrid[idx.r][idx.c][idx.h];
        mOutilerPoints.insert(mOutilerPoints.end(), ptids.begin(), ptids.end());
    }
    std::sort(mOutilerPoints.begin(), mOutilerPoints.end());
	
}

template<typename T>
void OutlierFilter<T>::clear()
{
    mOutilerPoints.clear();
    mGrid.clear();
}

template class OutlierFilter<float>;
template class OutlierFilter<double>;

}
