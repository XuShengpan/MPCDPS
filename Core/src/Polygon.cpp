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

#include "Polygon.h"
#include "VectorK.h"

namespace mpcdps {

Polygon::Polygon()
{

}

Polygon::~Polygon()
{

}

void Polygon::addPoint(double x, double y)
{
    _vertics.push_back(Point2d(x, y));
}

int Polygon::size() const
{
    return _vertics.size();
}

bool Polygon::isContains(double x, double y) const
{
    int n_vtx = _vertics.size();
    int i0 = 0;
    int i1 = 0;
    Vector2d v1;
    Vector2d v2;
    Point2d pt(x, y);
    double d0 = 0;
    double d = 0;
    for (int i = 0; i < n_vtx; ++i) {
        i0 = i;
        i1 = (i0 + 1) % (n_vtx);
        v1 = pt - _vertics[i0];
        v2 = pt - _vertics[i1];
        d = v1.determinate(v2);
        if (std::abs(d) < ZERO_D) {
            return false;  //on the polygon edeg
        }
        if (i == 0) {
            d0 = d;
        } else  if (d*d0 < ZERO_D) {
            return false;  //out of the polygon
        }   
    }
    return true;  //in the polygon
}

}