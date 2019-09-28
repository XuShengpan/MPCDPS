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

#include "OutlierFilter.h"
#include <algorithm>

namespace mpcdps {

OutlierFilter::OutlierFilter(void)
{
}

OutlierFilter::~OutlierFilter(void)
{
}

void OutlierFilter::initialize(const SmartArray2D<double, 3>& vtxAry, const Box3d& box, float distance_threshold)
{
    mVtxAry = vtxAry;
    mBox = box;
    mDistanceShreshold = distance_threshold;
    mGrid.initialize1(box.min(0), box.min(1), box.length(0), box.length(1), distance_threshold, distance_threshold);
    mGrid.initialize_z(box.min(2), box.length(2), distance_threshold);
}

void OutlierFilter::run()
{
	mOutilerPoints.clear();

    int r = 0;
    int c = 0;
    int h = 0;
    for (int i = 0; i < mVtxAry.size(); ++i) {
        r = mGrid.get_r(mVtxAry[i][1]);
        c = mGrid.get_c(mVtxAry[i][0]);
        h = mGrid.get_h(mVtxAry[i][2]);
        mGrid[r][c][h].push_back(i);
    }

	int rn = mGrid.rowCount(), cn = mGrid.colCount(), hn = mGrid.heightCount();
	mCellClassTag.resize(rn * cn * hn);
	mCellClassTag.reset(PC_Unknown);
	doWork();
}

void OutlierFilter::run(const std::vector<int>& ptIds)
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

	int rn = mGrid.rowCount(), cn = mGrid.colCount(), hn = mGrid.heightCount();
	mCellClassTag.resize(rn * cn * hn);
	mCellClassTag.reset(PC_Unknown);
	doWork();
}

std::vector<int> OutlierFilter::getOutlierPoints() const
{
	return mOutilerPoints;
}

void OutlierFilter::doWork()
{
	_rn = mGrid.rowCount();
	_cn = mGrid.colCount();
	_hn = mGrid.heightCount();

	std::vector<int> ptIds;
	for(int h=0;h<_hn;++h)  {
		for (int r = 0; r < _rn; ++r) {
			for (int c = 0; c < _cn; ++c) {
                if (!mGrid.is_empty(r, c, h)) {
                    if (isGridCellOutlier(r, c, h)) {
                        ptIds = mGrid[r][c][h];
                        mOutilerPoints.insert(mOutilerPoints.end(), ptIds.begin(), ptIds.end());
                    }
                }
			}
		}
	}
	std::sort(mOutilerPoints.begin(), mOutilerPoints.end());
}

bool OutlierFilter::isGridCellOutlier(int r,int c, int h)
{
    int lcd0 = getLinearId(r, c, h);
	if(mCellClassTag[lcd0] == PC_Outlier)
		return true;
	else if(mCellClassTag[lcd0] == PC_NotOutlier)
		return false;

    std::vector<int> ptIds = mGrid[r][c][h];
	if(ptIds.size() >= mNumShreshold) {
		mCellClassTag[lcd0] = PC_NotOutlier;
		return false;
	}

	int r1,c1,h1;
	int  numNotEmptyNergbor = 0;
	int rn = mGrid.rowCount(),cn = mGrid.colCount(),hn = mGrid.heightCount();

	for(int i=r-1;i<= r+1;++i)  {
		if(i<0 || i>= rn)  continue;
		for(int j=c-1;j<=c+1;++j) {
			if(j<0 || j>= cn)  continue;
			for(int k=h-1;k<=h+1;++k) {
				if(k<0 || k>= hn) continue;

				if( r == i && c == j && h == k) continue;
				int lcd = getLinearId(i,j,k);
				if(mCellClassTag[lcd] == PC_NotOutlier) {
					mCellClassTag[lcd0] = PC_NotOutlier;
					return false;
				}else if(mCellClassTag[lcd] == PC_Outlier) {
					mCellClassTag[lcd0] = PC_Outlier;
					return true;
				} else {
					if(!mGrid.is_empty(i,j,k)) {
                        ++ numNotEmptyNergbor;
						if(numNotEmptyNergbor == 1) {
							r1 = i;
							c1 = j;
							h1 = k;
						} else							
							break;
					}
				}
			}
		}
	}

	if(numNotEmptyNergbor == 0) {
		mCellClassTag[lcd0] = PC_Outlier;
		return true;
	} else if(numNotEmptyNergbor == 1) {
		if(mGrid[r1][c1][h1].size() < mNumShreshold) {
			if(getNotEmptyNeigborNum(r1,c1,h1) == 1) {
				mCellClassTag[lcd0] = PC_Outlier;
				int lcd = getLinearId(r1,c1,h1);
				mCellClassTag[lcd] = PC_Outlier;
				return true;
			}
		}
	} 
	
	mCellClassTag[lcd0] = PC_NotOutlier;
	int lcd = getLinearId(r1,c1,h1);
	mCellClassTag[lcd] = PC_NotOutlier;

	return false;
}

void OutlierFilter::clear()
{
	mOutilerPoints.clear();
	mGrid.clear();
	mCellClassTag.clear();
}

int OutlierFilter::getNotEmptyNeigborNum(int r,int c, int h) const
{
	int rn = mGrid.rowCount(),cn = mGrid.colCount(),hn = mGrid.heightCount();
	int n =0;
	for(int r1 = r-1;r1<r+2;++r1) {
		if(r1<0 || r1>=rn) continue;
		for(int c1 = c-1;c1<c+2;++c1) {
			if(c1<0) continue;
			if(c1>=cn) break;
			for(int h1 = h-1;h1<h+2;++h1) {
				if(h1<0) continue;
				if(h1>=hn) break;

				if(r1 == r && c1 == c && h1 == h) continue;
				if(!mGrid.is_empty(r1, c1, h1)) ++n;
			}
		}
	}
	return n;
}

}