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

#ifndef   MPCDPS_LINESEGMENT2_H
#define   MPCDPS_LINESEGMENT2_H

#include "Point2.h"
#include "Rect2.h"
#include "Line2.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /*Directional line segment of 2-dimension.*/
    template <typename T>
    class MPCDPS_CORE_ITEM LineSegment2
    {
    public:
        /* Default constructor.*/
        LineSegment2();

        /* Constructor: construct a line segment (p0, p1)*/
        LineSegment2(const Point2<T>& p0, const Point2<T>& p1);

        /*Destructor.*/
        ~LineSegment2();

        /*Test whether it intersects with another line segment.*/
        bool intersects(const LineSegment2& line) const;

        /* Test whether it intersects with a rect.*/
        bool intersects(const Rect2d& rect) const;

        //Get distance of the point to the line segment.
        double get_point_distance(const Point2<T>& p) const;

		double length() const;

        Point2<T>  _start;   /* Start point of the line segment.*/
        Point2<T>  _end;     /*End point of the line segment.*/
    };

    typedef LineSegment2 < float> LineSegment2f;
    typedef LineSegment2 < double> LineSegment2d;

	template <typename T> 
	LineSegment2<T> get_linesegment2(const std::vector<Point2<T> >& pointList, const Line2<T>& line);
}

#endif