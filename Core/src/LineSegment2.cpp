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

#include "LineSegment2.h"
#include "Line2.h"
#include <algorithm>

namespace mpcdps {

    double determinant(double v1, double v2, double v3, double v4)
    {  
        return (v1 * v3 - v2 * v4);
    }

    template <typename T>
    LineSegment2<T>::LineSegment2()
    {

    }

    template <typename T>
    LineSegment2<T>::~LineSegment2()
    {

    }

    template <typename T>
    LineSegment2<T>::LineSegment2(const Point2<T>& p0, const Point2<T>& p1)
        :_start(p0), _end(p1)
    {

    }

    template <typename T>
    bool LineSegment2<T>::intersects(const LineSegment2<T>& line, Point2<T>& intersection) const
    {
        {
            Line2<T> line1(_start, _end);
            Line2<T> line2(line._start, line._end);

            if (!line1.intersects(line2, intersection))
                return false;
        }

        Vector2<T> v1 = _end - _start;
        double d1 = v1.norm();
        v1 /= d1;

        Vector2<T> v = intersection - _start;
        double d = v.dot(v1);
        if (d < -1e-6 || d - d1 > 1e-6) {
            return false;
        }

        Vector2<T> v2 = line._end - line._start;
        double d2 = v2.norm();
        v2 /= d2;
        v = intersection - line._start;
        d = v.dot(v2);
        if (d < -1e-6 || d - d2 > 1e-6) {
            return false;
        }
        return true;
    }

    template <typename T>
    bool LineSegment2<T>::intersects(const Rect2d& rect) const
    {
        Vector2<T> v = _end - _start;
        double t = v.norm();
        v.normalize();
        double x0 = _start[0];
        double y0 = _start[1];
        double x1 = rect.min(0);
        double x2 = rect.max(0);
        double y1 = rect.min(1);
        double y2 = rect.max(1);

        if (std::abs(v.x()) <= ZERO_F) {
            if (x0 < x1 || x0 > x2) {
                return false;
            }
            double c = (y1 - y0) / v.y();
            double d = (y2 - y0) / v.y();
            if (c > d) {
                SWAP(c, d);
            }
            return c <= t && d >= 0;
        } else if (std::abs(v.y()) <= ZERO_F) {
            if (y0 < y1 || y0 > y2) {
                return false;
            }
            double a = (x1 - x0) / v.x();
            double b = (x2 - x0) / v.x();
            if (a > b) {
                SWAP(a, b);
            }
            return a <= t && b >= 0;
        } else {
            double a = (x1 - x0) / v.x();
            double b = (x2 - x0) / v.x();
            if (a > b) {
                SWAP(a, b);
            }
            double c = (y1 - y0) / v.y();
            double d = (y2 - y0) / v.y();
            if (c > d) {
                SWAP(c, d);
            }

            if (a > t || b < 0 || c > t || d < 0) {
                return false;
            }
            return (MAXV(a, c) - MINV(b, d) < b - a + d - c);
        }
    }

	template <typename T>
	double LineSegment2<T>::length() const
	{
		Vector2<T> v = _end - _start;
		return v.norm();
	}

    template <typename T>
    double LineSegment2<T>::get_point_distance(const Point2<T>& p) const
    {
        Vector2<T> u = _end - _start;
        double t = Vector2<T>(p - _start).dot(u) / u.norm();
        t = std::min<double>(std::max<double>(0, t), 1);
        Vector2<T> v = _start + u * t - p;
        return v.norm();
    }

	template <typename T>
	LineSegment2<T> get_linesegment2(const std::vector<Point2<T> >& pointList, const Line2<T>& line)
	{
		double e0 = DBL_MAX;
		double e1 = -DBL_MAX;
		double e;
		Point2<T> point = pointList[0];
		if (std::abs(line._b) > ZERO_F) {
			point[1] = (line._c - point[0] * line._a) / line._b;
		}
		else {
			point[0] = (line._c - point[1] * line._b) / line._a;
		}

		Vector2<T> dir = line.direction();
		for (int i = 1; i < pointList.size(); ++i) {
			e = Vector2<T>(pointList[i] - point).dot(dir);
			if (e < e0)  e0 = e;
			if (e > e1) e1 = e;
		}

		LineSegment2<T> line_seg;
		line_seg._start = point + dir * e0;
		line_seg._end = point + dir * e1;
		return line_seg;
	}

    template <typename T>
    bool LineSegment2<T>::intersects(const Circle<T>& circle, Point2<T>& intersection) const
    {
        Point2<T> A, B;
        if (circle.contains(_start)) {
            A = _start;
            if (circle.contains(_end)) {
                return false;
            } else {
                B = _end;
            }
        }

        if (circle.contains(_end)) {
            A = _end;
            if (circle.contains(_start)) {
                return false;
            } else {
                B = _start;
            }
        }

        Vector2<T> v = A - circle._center;
        T r1 = v.norm();
        v /= r1;

        Vector2<T> AB = B - A;
        double a = AB.norm();
        double b = AB.dot(v);
        double r = circle._radius;
        double a1 = a * a;
        double b1 = 2.0 * b * r1;
        double c1 = r1 * r1 - r * r;
        double delta = b1 * b1 - 4.0 * a1 * c1;
        double alpha = (-b1 + std::sqrt(delta)) / (2.0 * a1);
        intersection = A + AB*alpha;
        return true;
    }

    template MPCDPS_CORE_ITEM class LineSegment2<float>;
    template MPCDPS_CORE_ITEM class LineSegment2<double>;

	template MPCDPS_CORE_ITEM LineSegment2<float> get_linesegment2(const std::vector<Point2<float> >& pointList, const Line2<float>& line);
	template MPCDPS_CORE_ITEM LineSegment2<double> get_linesegment2(const std::vector<Point2<double> >& pointList, const Line2<double>& line);
}