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

#ifndef   MPCDPS_MEAN_SHIFT_CLUSTER_H
#define  MPCDPS_MEAN_SHIFT_CLUSTER_H

#include "MeanShift.h"
#include "PointCloudDistanceCluster.h"

namespace mpcdps {

	/**@brief
	* A distance cluster for point cloud.
	* T: data type of point cloud data buffer.
	* K: dimension of point.
	*/
	template<typename T, int K>
	class MeanShiftCluster: public PointCloudCluster<T, K>, public MeanShiftCore<T, K>
	{
	public:
		/* Default constructor.*/
		MeanShiftCluster() {}

		/* Destructor.*/
		~MeanShiftCluster() {}

        void initialize(const SmartArray2D<T, K>& vtx_array, const T vmin[K], const T vmax[K],
            std::vector<int> target_points = std::vector<int>())
        {
            PointCloudCluster<T, K>::initialize(vtx_array, vmin, vmax);
            MeanShiftCore<T, K>::initialize(vtx_array, vmin, vmax, target_points);
        }

        //default: 1.0
		void setModeDistance(double dist) { _mode_distance = dist; }

		/*Run the cluster. */
		virtual void run();

	protected:
		double _mode_distance = 1.0;
	};

#include "MeanShiftCluster.inl"
}

#endif