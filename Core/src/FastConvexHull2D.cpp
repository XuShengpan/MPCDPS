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

#include "FastConvexHull2D.h"
#include <cmath>
#include <stack>

#define  NEXT(i) ((i+1)%4)

namespace mpcdps {

	/*
	  @brief To test which side does point C lies in line AB.
	      >0: left side
	      =0: on the line
	      <0: right side
	*/
	template <typename Real>
	double whichSide(Real* ptA, Real* ptB, Real* ptC)
	{
		double x_AB = ptB[0] - ptA[0], y_AB = ptB[1] - ptA[1];
		double x_AC = ptC[0] - ptA[0], y_AC = ptC[1] - ptA[1];
		return x_AB*y_AC - x_AC*y_AB;
	}

	template <typename Real, uint K>
	FastConvexHull2D<Real,K>::FastConvexHull2D():mBoundNum(200)
	{

	}

	template <typename Real, uint K>
	FastConvexHull2D<Real,K>::~FastConvexHull2D()
	{

	}

	template <typename Real, uint K>
	void FastConvexHull2D<Real,K>::setVertexArray(const SmartArray2D<Real, K>& vtxAry)
	{
		mVtxAry = vtxAry;
	}

	template <typename Real, uint K>
	void FastConvexHull2D<Real,K>::setBoundNum(int n)
	{
		mBoundNum = n;
	}

	template <typename Real, uint K>
	std::vector<int> FastConvexHull2D<Real,K>::getConvexHull(const std::vector<int>& inputPtIds)
	{

		if(inputPtIds.size() < 4)
			return inputPtIds;

		int kp[4];
		double min_y = DBL_MAX,max_x = -DBL_MAX,max_y = -DBL_MAX,min_x = DBL_MAX;
		Real* pt;
		for(int i=0;i<inputPtIds.size();++i) {
			pt = mVtxAry[inputPtIds[i]];
			if(pt[0] > max_x) {
				max_x = pt[0];
				kp[1] = inputPtIds[i];
			}
			if(pt[0]<min_x) {
				min_x = pt[0];
				kp[3] = inputPtIds[i];
			}

			if(pt[1] > max_y) {
				max_y = pt[1];
				kp[2] = inputPtIds[i];
			}
			if(pt[1]<min_y) {
				min_y = pt[1];
				kp[0] = inputPtIds[i];
			}
		}

		std::vector<int> outputPtIds;

		std::vector<int> coner_vect[4];
		int j;
		for(std::vector<int>::const_iterator iter = inputPtIds.begin();iter != inputPtIds.end();++iter) {
			for(int i=0;i<4;++i) {
				j = NEXT(i);
				if(kp[i] == kp[j])  continue;
				if(whichSide(mVtxAry[kp[i]],mVtxAry[kp[j]], mVtxAry[*iter]) < 0) {
					coner_vect[i].push_back(*iter);
				}
			}
		}

		int tz1,tz2;
		bool isFirstRelatedToY = true;
		int minAlphaPtId,minBetaPtId;
		std::vector<int>::iterator iter_coner;
		std::stack<int> estack;

		for(int i=0;i<4;++i, isFirstRelatedToY = !isFirstRelatedToY) {
			tz1 = kp[i];
			tz2 = kp[NEXT(i)];

			if(tz1 == tz2) continue;
			outputPtIds.push_back(tz1);
			
			while(!coner_vect[i].empty()) {
				if(coner_vect[i].size() == 1) {
					outputPtIds.push_back(coner_vect[i][0]);
					break;
				} else if(coner_vect[i].size() > mBoundNum) {
					coner_vect[i] = getConvexHull(coner_vect[i]);
				}

				getMinAlphaBeta(coner_vect[i],tz1,tz2,isFirstRelatedToY,minAlphaPtId,minBetaPtId);
				if(minAlphaPtId == minBetaPtId) {
					outputPtIds.push_back(minAlphaPtId);
					break;
				} else {
					if(isFirstRelatedToY) {
						tz1 = minBetaPtId;
						tz2 = minAlphaPtId;
					} else {
						tz1 = minAlphaPtId;
						tz2 = minBetaPtId;
					}

					outputPtIds.push_back(tz1);
                    estack.push(tz2);

					std::vector<int> vect;
					for(iter_coner = coner_vect[i].begin();iter_coner != coner_vect[i].end();++iter_coner) {
						if(whichSide(mVtxAry[tz1],mVtxAry[tz2],mVtxAry[*iter_coner]) < 0)
							vect.push_back(*iter_coner);
					}

					coner_vect[i] = vect;
				}
			}

            while (!estack.empty()) {
                outputPtIds.push_back(estack.top());
                estack.pop();
            }

		}

		return outputPtIds;
	}

	template <typename Real, uint K>
	void FastConvexHull2D<Real,K>::getMinAlphaBeta(const std::vector<int>& ptIds,int firstCharactPtId,int secondCharactPtId,
		bool firstRelatedToY, int& minAlpha_ptId,int& minBeta_ptId)
	{
		Real* ptA, *ptB;
		if(firstRelatedToY) {
			ptA = mVtxAry[secondCharactPtId];
			ptB = mVtxAry[firstCharactPtId];
		} else {
			ptA = mVtxAry[firstCharactPtId];
			ptB = mVtxAry[secondCharactPtId];
		}

		double minAlpha = DBL_MAX, minBeta = DBL_MAX;
		minAlpha_ptId = -1;
		minBeta_ptId = -1;
		double alpha,beta;

		Real* pt;
		for(std::vector<int>::const_iterator iter = ptIds.begin();iter != ptIds.end();++iter) {
			pt = mVtxAry[*iter];
			alpha = std::abs((pt[0] - ptA[0])/(pt[1] - ptA[1]));
			beta = std::abs((pt[1]-ptB[1])/(pt[0]-ptB[0]));

			alpha = std::atan(alpha);
			beta = std::atan(beta);

			if(alpha < minAlpha) {
				minAlpha_ptId = *iter;
				minAlpha = alpha;
			}

			if(beta < minBeta) {
				minBeta_ptId = *iter;
				minBeta = beta;
			}
		}
	}

	template MPCDPS_CORE_ITEM class FastConvexHull2D<double,3>;
	template MPCDPS_CORE_ITEM class FastConvexHull2D<float,3>;
	template MPCDPS_CORE_ITEM class FastConvexHull2D<double,2>;
	template MPCDPS_CORE_ITEM class FastConvexHull2D<float,2>;


}






