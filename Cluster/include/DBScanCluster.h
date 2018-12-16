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

/*
Reference papar:
http://blog.csdn.net/google19890102/article/details/37656733
http://www.cnblogs.com/chaosimple/p/3164775.html
*/

#ifndef  MPCDPS_DBSCANCLUSTER_H
#define MPCDPS_DBSCANCLUSTER_H

#include "PointCloudCluster.h"
#include "KDTree.h"
#include <stack>

namespace mpcdps {

	/*
	   0 for no-target points.
	   1 for noise.
	*/
	template <typename T, int K>
	class DBScanCluster : public PointCloudCluster<T, K>
	{
	public:
		DBScanCluster();
		~DBScanCluster();

		/*Set the maximum distance between neighbors. */
		void setMaxDistance(T d) { _max_dist = d; }
		T getMaxDistance() const { return _max_dist; }

		void setMinNeighbor(int n) { _min_neighbor = n; }
		int   getMinNeighbor() const { return _min_neighbor; }

		/*Run the cluster. */
		virtual void run();

	protected:
		float _max_dist;
		int   _min_neighbor;

	};

#include "DBScanCluster.inl"

}

#endif