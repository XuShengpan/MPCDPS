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

        void initialize(const SmartArray2D<T, K>& vtx_array, T vmin[K], T vmax[K],
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

    template<typename T, int K>
    void MeanShiftCluster<T, K>::run()
    {
        if (_target_points.empty()) {
            _target_points = make_vector<int>(_vtx_array.size());
        }

        const int n_target = _target_points.size();
        SmartArray2D<T, K> modes(n_target);

#pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < n_target; ++i) {
            meanShift(_vtx_array[_target_points[i]], modes[i]);
        }

        {
            PointCloudDistanceCluster<T, K> dist_cluster;
            dist_cluster.initialize(modes, this->_vmin, this->_vmax);
            dist_cluster.setTargetPoints(make_vector<int>(n_target));
            dist_cluster.setMaxDistance(_mode_distance);
            dist_cluster.run();

            std::vector<uint> cls = dist_cluster.getClassVector();
            this->_cls_count = dist_cluster.getClassCount();
            this->_cls_ids.resize(this->_vtx_array.size(), 0);
            int ptid;
            for (int i = 0; i < n_target; ++i) {
                ptid = this->_target_points[i];
                this->_cls_ids[ptid] = cls[i];
            }
        }
    }
}

#endif