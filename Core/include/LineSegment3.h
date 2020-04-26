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

#ifndef  MPCDPS_LINESEGMENT3_H
#define  MPCDPS_LINESEGMENT3_H

#include "Point3.h"
#include "Line3.h"

namespace mpcdps {

    /**@brief Directional line segment of 3-dimension.*/
    template <typename T>
    class MPCDPS_CORE_ITEM LineSegment3
    {
    public:
        /**@brief Default constructor.*/
        LineSegment3();

        /**@brief  Constructor: construct a line segment (p0, p1)*/
        LineSegment3(const Point3<T>& p0, const Point3<T>& p1);

        /**@brief Destructor.*/
        ~LineSegment3();

		double length() const;
        
        /**@brief  Get distance of the point to the line segment. */
        double pointDistance(const Point3<T>& point) const;

        Point3<T>  _start;   /**@brief  Start point of the line segment.*/
        Point3<T>  _end;    /**@brief  End point of the line segment.*/
    };

    typedef LineSegment3<float> LineSegment3f;
    typedef LineSegment3<double> LineSegment3d;

    /**@brief Get a line segment with the given points and the line.*/
    template <typename T> 
    LineSegment3<T> get_linesegment3(const std::vector<Point3<T> >& points, const Line3<T>& line);
}

#endif