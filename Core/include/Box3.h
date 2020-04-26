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

#ifndef  MPCDPS_BOX3_H
#define MPCDPS_BOX3_H

#include "AABB.h"

namespace mpcdps {

    template<typename T>
    class Box3 : public AABB<T, 3>
    {
    public:
        Box3() : AABB<T, 3>() {}

        Box3(T xmin, T xmax, T ymin, T ymax, T zmin, T zmax)
        {
            this->_min[0] = xmin;
            this->_min[1] = ymin;
            this->_min[2] = zmin;
            this->_max[0] = xmax;
            this->_max[1] = ymax;
            this->_max[2] = zmax;
        }

        Box3(const Box3<T>& rth) : AABB<T, 3>(rth)
        {
        }

        Box3(T vmin[], T vmax[]) :AABB<T, 3>(vmin, vmax) {}

        Point3<T> center() const
        {
            Point3<T> pt;
            for (int i = 0; i < 3; ++i) {
                pt[i] = 0.5 * (this->_min[i] + this->_max[i]);
            }
            return pt;
        }
    };

    typedef Box3<float> Box3f;
    typedef Box3<double> Box3d;
}

#endif