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

#include "LineSegment3.h"

namespace mpcdps {

    template <typename T>
    LineSegment3<T>::LineSegment3()
    {

    }

    template <typename T>
    LineSegment3<T>::LineSegment3(const Point3<T>& p0, const Point3<T>& p1):_start(p0), _end(p1)
    {

    }

    template <typename T>
    LineSegment3<T>::~LineSegment3()
    {

    }

    template <typename T>
    double LineSegment3<T>::pointDistance(const Point3<T>& point) const
    {
        Vector3<T> e = _end - _start;
        Vector3<T> v = point - _start;
        e.normalize();
        Vector3<T> d = e.cross(v);
        return d.norm();
    }

	template <typename T>
	double LineSegment3<T>::length() const
	{
		Vector3<T> v = _end - _start;
		return v.norm();
	}

    template <typename T>
    LineSegment3<T> get_linesegment3(const std::vector<Point3<T> >& points, const Line3<T>& line)
    {
        Vector3<T> vect;
        double e = 0;
        double e0 = DBL_MAX;
        double e1 = -DBL_MAX;
        for (int i = 0; i < points.size(); ++i) {
            vect = points[i] - line._point;
            e = vect.dot(line._dir);
            if (e < e0) e0 = e;
            if (e > e1) e1 = e;
        }
        LineSegment3<T> line_seg;
        line_seg._start = line._point + e0 * line._dir;
        line_seg._end = line._point + e1 * line._dir;
        return line_seg;
    }

    template MPCDPS_CORE_ITEM class LineSegment3<float>;
    template MPCDPS_CORE_ITEM class LineSegment3<double>;

    template MPCDPS_CORE_ITEM
    LineSegment3<float> get_linesegment3(
        const std::vector<Point3<float> >& points, const Line3<float>& line);

    template MPCDPS_CORE_ITEM
    LineSegment3<double> get_linesegment3(
        const std::vector<Point3<double> >& points, const Line3<double>& line);

}