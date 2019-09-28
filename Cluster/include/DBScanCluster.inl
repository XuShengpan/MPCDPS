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
    if (this->_target_points.empty()) {
        this->_target_points = make_vector<int>(this->_vtx_array.size());
    }

    if (this->_seeds.empty()) {
        this->_seeds = this->_target_points;
    }

    KDTree<T, K> kdtree;
    double nodesize = 2.0;
    std::vector<T> ns(K, 0);
    for (int i = 0; i < K; ++i) {
        ns[i] = nodesize;
    }

    kdtree.setElements(_vtx_array);
    kdtree.build(_target_points, _vmin, _vmax, &ns[0]);

    std::stack<int> stk;
    std::vector<bool> tag(_vtx_array.size(), false);  //process

    std::vector<int> cls_ids(_vtx_array.size(), -1);
    int next_cls_id = 1;  //0 is for outliers

    for (size_t i = 0; i < _seeds.size(); ++i) {
        int pt_id = _seeds[i];
        if (tag[pt_id]) {
            continue;
        }

        {
            std::vector<double> dist2s;
            std::vector<int> neigbs = kdtree.searchRadius(this->_vtx_array[pt_id], this->_max_dist, dist2s);
            if (neigbs.size() < this->_min_neighbor) {
                cls_ids[pt_id] = 0;
                tag[pt_id] = true;
                continue;
            } else if (neigbs.size() == this->_min_neighbor) {
                cls_ids[pt_id] = 0;
                continue;
            }
        }

        int cls_id = next_cls_id++;
        stk.push(pt_id);

        std::vector<bool> instk(_vtx_array.size(), false);
        instk[pt_id] = true;

        while (!stk.empty()) {
            pt_id = stk.top();
            stk.pop();

            std::vector<double> dist2s;
            std::vector<int> neigbs = kdtree.searchRadius(this->_vtx_array[pt_id], this->_max_dist, dist2s);
            
            if (neigbs.size() < this->_min_neighbor) {  //outliers
                cls_ids[pt_id] = 0;
                tag[pt_id] = true;
            } else if(neigbs.size() == this->_min_neighbor) {  //boundaries
                cls_ids[pt_id] = cls_id;
                tag[pt_id] = true;
            } else {  //kernels
                cls_ids[pt_id] = cls_id;
                tag[pt_id] = true;
                for (size_t j = 0; j < neigbs.size(); ++j) {
                    pt_id = neigbs[j];
                    if (!tag[pt_id]) {
                        if (!instk[pt_id]) {
                            stk.push(pt_id);
                            instk[pt_id] = true;
                        }
                    }
                }
            }
        }
    }

    _cls_count = next_cls_id;
    _cls_ids = cls_ids;
}