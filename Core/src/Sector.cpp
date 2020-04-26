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

#include "Sector.h"
#include "LineSegment2.h"

//reference paper: https://zhuanlan.zhihu.com/p/23903445

namespace mpcdps {

    bool is_intersect_sector2circle(const Sector& sector, const Circle<double>& circle)
    {
        Vector2d d = circle._center - sector.center;
        double rsum = sector.radius + circle._radius;
        if (d[0] * d[0] + d[1] * d[1] > rsum * rsum) {
            return false;
        }

        double px = d.dot(sector.direction);
        double py = std::abs(d.dot(Vector2d(-sector.direction.y(), sector.direction.x())));
        
        if (px > d.norm() * std::cos(sector.theta)) {
            return true;
        }

        Vector2d q = sector.radius * Vector2d(std::cos(sector.theta), std::sin(sector.theta));
        LineSegment2d line;
        line._start = Point2d(0, 0);
        line._end = Point2d(q.x(), q.y());
        return line.get_point_distance(Point2d(px, py)) < circle._radius;
    }

}