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

#ifndef MPCDPS_RECT3_H
#define MPCDPS_RECT3_H

#include "Point3.h"
#include "VectorK.h"

namespace mpcdps {

    template <typename T>
    class Rect3
    {
    public:
        Rect3()
        {
        }

        Rect3(const Rect3<T>& rth)
        {
            _center = rth._center;
            _extend[0] = rth._extend[0];
            _extend[1] = rth._extend[1];
            _axis[0] = rth._axis[0];
            _axis[1] = rth._axis[1];
        }

        ~Rect3()
        {
        }

        bool contains(const Point3<T>& point, T eps = T(0.01)) const
        {
            Vector3<T> v = point - _center;
            Vector3<T> normal = _axis[0].cross(_axis[1]);
            normal.normalize();
            if (std::abs(v.dot(normal)) > eps || 
                std::abs(std::abs(v.dot(_axis[0])) - _extend[0]) > eps
                std::abs(std::abs(v.dot(_axis[1])) - _extend[1]) > eps) {
                return false;
            }
            return true;
        }

        Point3<T> _center;
        Vector3<T> _axis[2];  //unit vector
        T _extend[2];  // > 0
    };

    typedef Rect3<float> Rect3f;
    typedef Rect3<double> Rect3d;
}

#endif