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

#ifndef MPCDPS_POLYGON_H
#define MPCDPS_POLYGON_H

#include "Point2.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {
	
class MPCDPS_CORE_ITEM Polygon
{
public:
    Polygon();
    ~Polygon();

    void addPoint(double x, double y);
    bool isContains(double x, double y) const;
    int   size() const;

protected:
    std::vector<Point2d> _vertics;
};

}
#endif