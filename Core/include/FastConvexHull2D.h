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

#ifndef MPCDPS_FASTCONVEXHULL2D_H
#define MPCDPS_FASTCONVEXHULL2D_H

#include <vector>
#include "SmartArray2D.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps{

template <typename Real, uint K>
class MPCDPS_CORE_ITEM FastConvexHull2D
{
public:
	FastConvexHull2D();
	~FastConvexHull2D();

	void setVertexArray(const SmartArray2D<Real, K>& vtxAry);
	void setBoundNum(int n);

	//return convex point ids along CCW
	std::vector<int> getConvexHull(const std::vector<int>& inputPtIds);

protected:
	void getMinAlphaBeta(const std::vector<int>& ptIds,int firstCharactPtId,int secondCharactPtId,bool firstRelatedToY,
		   int& minAlpha_ptId,int& minBeta_ptId);

private:
	SmartArray2D<Real, K> mVtxAry;
	int mBoundNum;   //default: 200

};

}

#endif