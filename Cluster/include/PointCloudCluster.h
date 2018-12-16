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

#ifndef MPCDPS_POINTCLOUDCLUSTER_H
#define MPCDPS_POINTCLOUDCLUSTER_H

/*
===============================================================================

This file is a part of  library MPCDPS (Massive Point Cloud Data Processing System) and it is developed
by Xu Shengpan, anyone can't duplicate or share it without permission from the author.
For any question please contact jack_1227x@163.com.

Copyright Xu Shengpan, all rights reserved.

===============================================================================
*/

#include <vector>
#include "SmartArray2D.h"

namespace mpcdps {

	/**@brief
	* A distance cluster for point cloud.
	* T: data type of point cloud data buffer.
	* K: dimension of point.
	*/
	template<typename T, int K>
	class PointCloudCluster
	{
	public:
		typedef SmartArray2D<T, K> VertexArrayType;  /*Type define for VertexArrayType. */

		/* Default constructor.*/
		PointCloudCluster();

		/* Destructor.*/
		~PointCloudCluster();

		/* Initialize for the cluster.*/
		void initialize(const SmartArray2D<T, K>& vtx_array, T vmin[K], T vmax[K]);

		/* Set target ids of points for clustering.*/
		void setTargetPoints(const std::vector<int>& point_ids);

		/* Set seeds to be clustered.*/
		void setSeeds(const std::vector<int>& seeds) { _seeds = seeds; }

		/*Run the cluster. */
		virtual void run() = 0;

		/*Get class count. */
		int getClassCount() const { return _cls_count; }

		/*Get class points.
		* cls = 0, 1, ... , n-1;  n = class_count.
		* cls = 0 is for the points that are not in target_points.
		*/
		std::vector<int> getClassPoints(int cls) const;

		/*Clear the cluster. */
		virtual void clear();

		std::vector<uint> getClassVector() const { return _cls_ids; }

	protected:
		VertexArrayType _vtx_array;  /*Vertex array. */
		std::vector<int> _target_points;   /*Target points for clustering. */
		T _vmin[K];
		T _vmax[K];
		std::vector<uint> _cls_ids;
		int _cls_count;

		std::vector<int> _seeds;
	};

#include "PointCloudCluster.inl"

}

#endif
