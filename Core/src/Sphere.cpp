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

#include "Sphere.h"

namespace mpcdps {

    template <typename T>
    Sphere<T>::Sphere():_radius(0)
    {

    }

    template <typename T>
    Sphere<T>::Sphere(const Sphere<T>& rth):_center(rth._center), _radius(rth._radius)
    {

    }

    template <typename T>
    Sphere<T>::~Sphere()
    {

    }

    template <typename T>
    bool Sphere<T>::intersects(const Sphere<T>& obj) const
    {
        Vector3<T> v = obj._center - _center;
        return v.norm() < _radius + obj._radius;
    }

    template <typename T>
    bool Sphere<T>::contains(const Sphere<T>& obj) const
    {
        Vector3<T> v = obj._center - _center;
        return  _radius >= v.norm() + obj._radius;
    }

    template <typename T>
    Sphere<T>& Sphere<T>::combine(const Sphere<T>& obj)
    {
        if (!contains(obj)) {
            if (obj.contains(*this)) {
                _center = obj._center;
                _radius = obj._radius;
            } else {
                Point3<T> b = _center;
                Point3<T>    e = obj._center;
                Vector3<T> v = e - b;
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
    Sphere<T> Sphere<T>::combined(const Sphere<T>& obj) const
    {
        if (contains(obj)) {
            return *this;
        } else if (obj.contains(*this)) {
            return obj;
        } else {
            Point3<T> b = _center;
            Point3<T> e = obj._center;
            Vector3<T> v = e - b;
            double d = v.norm();
            v.normalize();
            b = b - _radius * v;
            e = e + obj._radius * v;

            Sphere<T> sph;
            sph._radius = 0.5 * (_radius + d + obj._radius);
            sph._center = b + sph._radius * v;

            return sph;
        }
    }

	template <typename T>
	bool Sphere<T>::intersects(const Box3<T>& box) const
	{
		double r1 = box.radius();
		Vector3<T> v = box.center() - _center;
		double d = v.norm();
		if (d >= r1 + _radius)
			return false;
		double a = box.length(0) * 0.5, b = box.length(1) * 0.5, c = box.length(2) * 0.5;
		T ex[] = { 1, 0, 0 };
		T ey[] = { 0, 1, 0 };
		T ez[] = { 0, 0, 1 };
		double a1 = v.dot(VectorK<T, 3>(ex)), b1 = v.dot(VectorK<T, 3>(ey)), c1 = v.dot(VectorK<T,3>(ez));
		if (d < _radius + std::sqrt(a1 * a1 + b1 * b1 + c1 * c1))
			return true;
		if (std::abs(box.min(0) - _center[0]) < _radius + a)
			return true;
		if (std::abs(box.max(0) - _center[0]) < _radius + a)
			return true;
		if (std::abs(box.min(1) - _center[1]) < _radius + b)
			return true;
		if (std::abs(box.max(1) - _center[1]) < _radius + b)
			return true;
		if (std::abs(box.min(2) - _center[2]) < _radius + c)
			return true;
		if (std::abs(box.max(2) - _center[2]) < _radius + c)
			return true;
		return false;
	}

    template MPCDPS_CORE_ITEM class Sphere<float>;
    template MPCDPS_CORE_ITEM class Sphere<double>;
}