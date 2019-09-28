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

#ifndef MPCDPS_RECT2_H
#define MPCDPS_RECT2_H

#include "AABB.h"

namespace mpcdps {

    template<typename T>
    class Rect2 : public AABB<T, 2>
    {
    public:
        Rect2() : AABB<T, 2>() {}

        Rect2(T xmin, T xmax, T ymin, T ymax)
        {
            this->_min[0] = xmin;
            this->_min[1] = ymin;
            this->_max[0] = xmax;
            this->_max[1] = ymax;
        }

        Rect2(const Rect2<T>& rth) : AABB<T, 2>(rth)
        {
        }

        Rect2(const Tuple<T, 2>& cnt, T ex, T ey)
        {
            this->_min[0] = cnt[0] - ex;
            this->_min[1] = cnt[1] - ey;
            this->_max[0] = cnt[0] + ex;
            this->_max[1] = cnt[1] + ey;
        }

        Rect2(T vmin[], T vmax[]) : AABB<T, 2>(vmin, vmax) {}

        Point2<T> center() const
        {
            Point2<T> pt;
            for (int i = 0; i < 2; ++i) {
                pt[i] = 0.5 * (this->_min[i] + this->_max[i]);
            }
            return pt;
        }
    };

    typedef Rect2<float> Rect2f;
    typedef Rect2<double> Rect2d;
}

#endif