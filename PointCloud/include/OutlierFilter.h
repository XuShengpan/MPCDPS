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

#ifndef MPCDPS_OUTLIERFILTER_H
#define MPCDPS_OUTLIERFILTER_H

#include <vector>
#include <Box3.h>
#include <SmartArray2D.h>
#include <SmartArray.h>
#include "VoxelGrid.h"

namespace mpcdps {

template<typename T=double>
class OutlierFilter
{
public:
	OutlierFilter(void);
	virtual ~OutlierFilter(void);

	void initialize(const SmartArray2D<T, 3>& vtxAry, const Box3<T>& box, float distance_threshold);

    //defaut:  2
    void setNumberThreshold(int n) { mNumShreshold = n; }

	void run();
	void run(const std::vector<int>& ptIds);

	std::vector<int> getOutlierPoints() const;

	void clear();

private:
	void doWork();
    uint getLinearId(int r, int c, int h) const
    {
        return h * _rn * _cn + r * _cn + c;
    }

protected:
    SmartArray2D<T, 3> mVtxAry;
	Box3<T> mBox;
    VoxelGrid<std::vector<int> > mGrid;
	std::vector<int>  mOutilerPoints;

	int   mNumShreshold = 2;
	float mDistanceShreshold = 1;

protected:
    uint _rn = 0;
    uint _cn = 0;
    uint _hn = 0;

};

}


#endif
