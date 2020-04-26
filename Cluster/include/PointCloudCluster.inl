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

template<typename T, int K>
PointCloudCluster<T, K>::PointCloudCluster()
{
}

template<typename T, int K>
PointCloudCluster<T, K>::~PointCloudCluster()
{
}

template<typename T, int K>
void PointCloudCluster<T, K>::initialize(
	const SmartArray2D<T, K>& vtx_array, const T vmin[K], const T vmax[K])
{
	_vtx_array = vtx_array;
	for (int i = 0; i < K; ++i) {
		_vmin[i] = vmin[i];
		_vmax[i] = vmax[i];
	}
}

template<typename T, int K>
void PointCloudCluster<T, K>::clear()
{
	_target_points.clear();
	_cls_ids.clear();
	_cls_count = 0;
	_seeds.clear();
}

template<typename T, int K>
std::vector<int> PointCloudCluster<T, K>::getClassPoints(int cls) const
{
	std::vector<int> ptids;
	int n = _vtx_array.size();
	for (int i = 0; i < n; ++i) {
		if (_cls_ids[i] == cls)
			ptids.push_back(i);
	}
	return ptids;
}

template<typename T, int K>
void PointCloudCluster<T, K>::setTargetPoints(const std::vector<int>& points)
{
	_target_points = points;
}