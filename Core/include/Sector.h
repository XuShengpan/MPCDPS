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

#ifndef MPCDPS_SECTOR_H
#define MPCDPS_SECTOR_H

#include "Point2.h"
#include "Circle.h"
#include "VectorK.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    struct Sector
    {
        Point2d    center;
        Vector2d  direction;  //center line direction, unit vector
        double  radius;
        double  theta;  //half angle in rad
    };

    MPCDPS_CORE_ITEM bool is_intersect_sector2circle(const Sector& sector, const Circle<double>& circle);

}


#endif