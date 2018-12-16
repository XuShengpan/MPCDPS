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

#include "Circle.h"

namespace mpcdps {

    template <typename T>
    Circle<T>::Circle():_radius(0)
    {

    }

    template <typename T>
    Circle<T>::Circle(const Circle& obj):_center(obj._center), _radius(obj._radius)
    {
    }

    template <typename T>
    Circle<T>::~Circle()
    {

    }

    template <typename T>
    bool Circle<T>::intersects(const Circle<T>& obj) const
    {
        Vector2<T> v = obj._center - _center;
        return v.norm() < _radius + obj._radius;
    }

    template <typename T>
    bool Circle<T>::contains(const Circle<T>& obj) const
    {
        Vector2<T> v = obj._center - _center;
        return v.norm() + obj._radius < _radius;
    }

	template <typename T>
	bool Circle<T>::contains(const Point2<T>& pt) const
	{
		Vector2<T> v = pt - _center;
		return (v.norm() < _radius);
	}

    template <typename T>
    Circle<T>& Circle<T>::combine(const Circle<T>& obj)
    {
        if (!contains(obj)) {
            if (obj.contains(*this)) {
                _center = obj._center;
                _radius = obj._radius;
            } else {
                Point2<T> b = _center;
                Point2<T> e = obj._center;
                Vector2<T> v = e - b;
                double d = v.norm();
                v.normalize();
                b = b - _radius * v;
                e = e + obj._radius * v;
                _radius = 0.5 * (_radius + d + obj._radius);
                _center = b + _radius * v;
            }
        }
        return *this;
    }

    template <typename T>
    Circle<T> Circle<T>::combined(const Circle<T>& obj) const
    {
        if (contains(obj)) {
            return *this;
        }
        else if (obj.contains(*this)) {
            return obj;
        }
        else {
            Point2<T> b = _center;
            Point2<T> e = obj._center;
            Vector2<T> v = e - b;
            double d = v.norm();
            v.normalize();
            b = b - _radius * v;
            e = e + obj._radius * v;
            Circle<T> sph;
            sph._radius = 0.5 * (_radius + d + obj._radius);
            sph._center = b + sph._radius * v;
            return sph;
        }
    }

	template <typename T>
	bool Circle<T>::intersects(const Rect2<T>& rect) const
	{
		double r1 = rect.radius();
		Vector2<T> v = rect.center() - _center;
		double d = v.norm();
		if (d >= r1 + _radius)
			return false;
		double a = rect.length(0) * 0.5, b = rect.length(1) * 0.5;
		T ex[] = { 1, 0 };
		T ey[] = { 0, 1 };
		double a1 = v.dot(VectorK<T, 2>(ex)), b1 = v.dot(VectorK<T, 2>(ey));
		if (d < _radius + std::sqrt(a1 * a1 + b1 * b1))
			return true;
		if (std::abs(rect.min(0) - _center[0]) < _radius + a)
			return true;
		if (std::abs(rect.max(0) - _center[0]) < _radius + a)
			return true;
		if (std::abs(rect.min(1) - _center[1]) < _radius + b)
			return true;
		if (std::abs(rect.max(1) - _center[1]) < _radius + b)
			return true;
		return false;
	}

    template MPCDPS_CORE_ITEM class Circle<float>;
    template MPCDPS_CORE_ITEM class Circle<double>;
}