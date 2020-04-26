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

#ifndef  MPCDPS_AABB_H
#define  MPCDPS_AABB_H

#include <limits>
#include "Tuple.h"
#include "Point2.h"
#include "Point3.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /* Axis aligned bounding box (AABB) of K-dimension.*/
    template <typename  T, int K>
    class  AABB
    {
    public:

        /*Default constructor. */
        AABB()
        {
        }

        /*Constructor. */
        AABB(T vmin[], T vmax[])
        {
            for (int i = 0; i < K; ++i) {
                _min[i] = vmin[i];
                _max[i] = vmax[i];
            }
        }

        /*Copy constructor. */
        AABB(const AABB<T, K>& box)
        {
            for (int i = 0; i < K; ++i) {
                _min[i] = box.min(i);
                _max[i] = box.max(i);
            }
        }

        /*Destructor. */
        ~AABB()
        {

        }

        /*= operator: initialize as the given box.*/
        void operator = (const AABB<T, K>& box)
        {
            for (int i = 0; i < K; ++i) {
                _min[i] = box.min(i);
                _max[i] = box.max(i);
            }
        }

        const T* min_ptr() const { return _min; }
        const T* max_ptr() const { return _max; }

        /* Get min value in k dimension.*/
        T  min(int k)  const { return _min[k]; }

        /* Get max value in k dimension.*/
        T  max(int k) const { return _max[k]; }

        /* Get length in k dimension.*/
        T  length(int k)  const { return (_max[k] - _min[k]); }

        /* Set min value in k dimension.*/
        void setMin(int k, T v) { _min[k] = v; }

        /* Set max value in k dimension.*/
        void setMax(int k, T v) { _max[k] = v; }

        /* Test whether the box is valid.*/
        bool  isValid()  const
        {
            for (int i = 0; i < K; ++i) {
                if (_max[i] < _min[i]) {
                    return false;
                }
            }
            return true;
        }

        /* Get the center of the box.*/
        Tuple<T, K> center() const
        {
            Tuple<T, K> pt;
            for (int i = 0; i < K; ++i) {
                pt[i] = 0.5 * (_min[i] + _max[i]);
            }
            return pt;
        }

        /* Get the radius of the box.*/
        double radius() const
        {
            double dx = 0;
            double sdx2 = 0;
            for (int i = 0; i < K; ++i) {
                dx = _max[i] - _min[i];
                sdx2 += dx * dx;
            }
            return  0.5 * std::sqrt(sdx2);
        }

        /* To test whether the box contains the point.*/
        bool contains(const Tuple<T, K>& pt) const
        {
            for (int i = 0; i < K; ++i) {
                if (_min[i] > pt[i] || _max[i] < pt[i]) {
                    return false;
                }
            }
            return true;
        }

        /* To test whether the box contains the box.*/
        bool  contains(const AABB<T, K>& box) const
        {
            for (int i = 0; i < K; ++i) {
                if (_min[i] > box.min(i) || _max[i] < box.max(i)) {
                    return false;
                }
            }
            return true;
        }

        /* To test whether the box intersects with the box.*/
        bool  intersects(const AABB<T, K>& box) const
        {
            for (int i = 0; i < K; ++i) {
                if (_min[i] > box.max(i) || _max[i] < box.min(i)) {
                    return false;
                }
            }
            return true;
        }

        /* To get intersected box. If the two box are not intersected, the returned box is invalid.*/
        AABB<T, K> intersected(const AABB<T, K>& box) const
        {
            T vmin[K];
            T vmax[K];

            for (int i = 0; i < K; ++i) {
                vmin[i] = std::max(_min[i], box.min(i));
                vmax[i] = std::min(_max[i], box.max(i));
            }

            return AABB(vmin, vmax);
        }

        /* Combined with another box.*/
        AABB<T, K> combined(const AABB<T, K>& box)  const
        {
            T vmin[K];
            T vmax[K];

            for (int i = 0; i < K; ++i) {
                vmin[i] = std::min(_min[i], box.min(i));
                vmax[i] = std::max(_max[i], box.max(i));
            }

            return AABB(vmin, vmax);
        }

        /* Combine with another box.*/
        AABB<T, K>& combine(const AABB<T, K>& box)
        {
            for (int i = 0; i < K; ++i) {
                if (box._min[i] < _min[i]) {
                    _min[i] = box._min[i];
                }
                if (box._max[i] > _max[i]) {
                    _max[i] = box._max[i];
                }
            }
            return *this;
        }

    protected:
        T _min[K];
        T _max[K];
    };
}

#endif // MPCDPS_H
