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

template <typename T, int K>
DBScanCluster<T, K>::DBScanCluster() :_max_dist(1), _min_neighbor(5)
{

}

template <typename T, int K>
DBScanCluster<T, K>::~DBScanCluster()
{

}

template <typename T, int K>
void DBScanCluster<T, K>::run()
{
	if (_target_points.empty()) {
		_target_points = make_vector<int>(_vtx_array.size());
	}

	if (_seeds.empty())
		_seeds = _target_points;

	KDTree<T, K> kdtree;
	double nodesize = 2.0;
	SmartArray<T> ns(K);
	for (int i = 0; i < K; ++i) {
		ns[i] = nodesize;
	}

	kdtree.setElements(_vtx_array);
	kdtree.build(_target_points, _vmin, _vmax, ns.buffer());

	std::stack<int> stk;
	std::vector<bool> tag(_vtx_array.size(), false);
	int pt_id;
	int pt_id1;

	std::vector<double> dist2s;
	std::vector<int> neigbs;

	std::vector<uint> cls_ids(_vtx_array.size(), 0);
	uint cls_id;
	uint next_cls_id = 2;

	int n_cls = 0;
	for (size_t i = 0; i < _seeds.size(); ++i) {
		pt_id = _seeds[i];
		if (tag[pt_id]) {
			continue;
		}
		neigbs = kdtree.searchRadius(_vtx_array[pt_id], _max_dist, dist2s);
		if (neigbs.size() < _min_neighbor) {
			cls_ids[pt_id] = 1;
			continue;
		}

		cls_id = next_cls_id++;
		cls_ids[pt_id] = cls_id;
		stk.push(pt_id);
		while (!stk.empty()) {
			pt_id = stk.top();
            stk.pop();
			if(tag[pt_id])
				continue;
			neigbs = kdtree.searchRadius(_vtx_array[pt_id], _max_dist, dist2s);
			tag[pt_id] = true;

			if (neigbs.size() >= _min_neighbor) {
				for (int j = 0; j < neigbs.size(); ++j) {
					pt_id1 = neigbs[j];
					if (!tag[pt_id1]) {
						stk.push(pt_id1);
						cls_ids[pt_id1] = cls_id;
					}
				}
			} else {
				if (cls_ids[pt_id] <= 1)
					cls_ids[pt_id] = cls_id;
			}
		}
	}

	_cls_count = next_cls_id;
	_cls_ids = cls_ids;
}