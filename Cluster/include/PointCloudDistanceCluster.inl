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

template<typename T, int K>
PointCloudDistanceCluster<T, K>::PointCloudDistanceCluster()
{
}

template<typename T, int K>
PointCloudDistanceCluster<T, K>::~PointCloudDistanceCluster()
{
}

template<typename T, int K>
void PointCloudDistanceCluster<T, K>::run()
{
	if (_target_points.empty()) {
		_target_points = make_vector<int>(_vtx_array.size());
	}

	if (_seeds.empty())
		_seeds = _target_points;

	KDTree<T, K> kdtree;
	double nodesize = 2.0;
	std::vector<T> ns(K, 0);
	for (int i = 0; i < K; ++i) {
		ns[i] = nodesize;
	}

	kdtree.setElements(_vtx_array);
	kdtree.build(_target_points, _vmin, _vmax, &ns[0]);

	std::stack<int> stk;
	SmartArray<bool> tag(_vtx_array.size());
	tag.reset(false);
	int pt_id;

	std::vector<double> dist2s;
	std::vector<int> neigbs;

	std::vector<int> cls_ids(_vtx_array.size(), -1);
	int next_cls_id = 0;

	for (size_t i = 0; i < _seeds.size(); ++i) {
		pt_id = _seeds[i];

		if (tag[pt_id]) {
			continue;
		}

		int cls_id = next_cls_id++;
		cls_ids[pt_id] = cls_id;

		stk.push(pt_id);
		tag[pt_id] = true;
		while (!stk.empty()) {
            pt_id = stk.top();
            stk.pop();
			dist2s.clear();
			neigbs = kdtree.searchRadius(_vtx_array[pt_id], _max_dist, dist2s);

			for (size_t j = 0; j < neigbs.size(); ++j) {
				pt_id = neigbs[j];
				if (!tag[pt_id]) {
					cls_ids[pt_id] = cls_id;
					stk.push(pt_id);
					tag[pt_id] = true;
				}
			}
		}
	}

	_cls_count = next_cls_id;
	_cls_ids = cls_ids;
}