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

#ifndef MPCDPS_POINTCLOUDDISTANCECLUSTER_H
#define MPCDPS_POINTCLOUDDISTANCECLUSTER_H

#include <iostream>
#include <stack>
#include "PointCloudCluster.h"
#include "KDTree.h"
#include "SmartArray.h"

namespace mpcdps {

    /**@brief
    * A distance cluster for point cloud.
    * T: data type of point cloud data buffer.
    * K: dimension of point.
    */
    template<typename T, int K>
    class PointCloudDistanceCluster: public PointCloudCluster<T, K>
    {
    public:
        /* Default constructor.*/
        PointCloudDistanceCluster();

        /* Destructor.*/
        ~PointCloudDistanceCluster();

        /*Set the maximum distance between neighbors. */
        void setMaxDistance(T d) { _max_dist = d; }
        T getMaxDistance() const { return _max_dist; }

        /*Run the cluster. */
        virtual void run();

    protected:
        T _max_dist;
    };

#include "PointCloudDistanceCluster.inl"

}

#endif
