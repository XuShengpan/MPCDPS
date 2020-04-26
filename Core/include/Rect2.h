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

    template<typename T>
    class RRect2
    {
    public:
        RRect2() {}
        RRect2(const Rect2<T>& rect)
        {
            _center = rect.center();
            _ex = rect.length(0) * 0.5;
            _ey = rect.length(1) * 0.5;
        }

        T radius() const
        {
            return std::sqrt(_ex * _ex + _ey * _ey);
        }

        void scale(T scalar)
        {
            _ex *= scalar;
            _ey *= scalar;
        }

        void scale(T sx, T sy)
        {
            _ex *= sx;
            _ey *= sy;
        }

        void move(const Vector2<T>& vect)
        {
            _center += vect;
        }

        Rect2<T> getRect() const
        {
            T xmin = _center[0] - _ex;
            T xmax = _center[0] + _ex;
            T ymin = _center[1] - _ey;
            T ymax = _center[1] + _ey;
            return Rect2<T>(xmin, xmax, ymin, ymax);
        }

        T length(int k) const
        {
            if (k == 0)
                return 2.0 * _ex;
            else
                return 2.0 * _ey;
        }

        Point2<T> _center;
        T _ex = 0;
        T _ey = 0;
    };

    typedef Rect2<float> Rect2f;
    typedef Rect2<double> Rect2d;

    typedef RRect2<float> RRect2f;
    typedef RRect2<double> RRect2d;
}

#endif