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

#ifndef MPCDPS_BOX_GETTER_H
#define MPCDPS_BOX_GETTER_H

#include "Box3.h"

namespace mpcdps {

    struct BoxGetter
    {
        void add_point(double x, double y, double z)
        {
            if (x < _xmin)  _xmin = x;
            if (x > _xmax)  _xmax = x;
            if (y < _ymin)  _ymin = y;
            if (y > _ymax)  _ymax = y;
            if (z < _zmin)  _zmin = z;
            if (z > _zmax)  _zmax = z;
        }

        template<typename Real>
        Box3<Real> getBox() const
        {
            return Box3<Real>(_xmin, _xmax, _ymin, _ymax, _zmin, _zmax);
        }

        double _xmin = DBL_MAX;
        double _xmax = -DBL_MAX;
        double _ymin = DBL_MAX;
        double _ymax = -DBL_MAX;
        double _zmin = DBL_MAX;
        double _zmax = -DBL_MAX;
    };
}

#endif