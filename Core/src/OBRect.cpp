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

#include "OBRect.h"
#include <math.h>
#include "FastConvexHull2D.h"
#include "Matrix.h"

namespace mpcdps {

	template <typename T>
	void getProjection(const std::vector<Point2<T> >& points, Point2<T> center,
		const Vector2<T>& axis, double& e0, double & e1)
	{
		Vector2<T> v;
		e0 = DBL_MAX;
		e1 = -DBL_MAX;
		double e;
		for (int i = 0; i < points.size(); ++i) {
			v = points[i] - center;
			e = v.dot(axis);
			if (e < e0) e0 = e;
			if (e > e1) e1 = e;
		}
	}

	template<typename T>
	OBRect<T>::OBRect()
	{
		_extents[0] = 0;
		_extents[1] = 0;
	}

	template<typename T>
	OBRect<T>::~OBRect()
	{

	}

	template<typename T>
	T OBRect<T>::getRadius() const
	{
		return  std::sqrt(double(_extents[0] * _extents[0] + _extents[1] * _extents[1]));
	}

	template<typename T>
	bool OBRect<T>::contains(const Point2<T>& point) const
	{
		Vector2<T> v = point - _center;
		double t = v.dot(_axises[0]);
		if (std::abs(t) > _extents[0])
			return false;
		t = v.dot(_axises[1]);
		if (std::abs(t) > _extents[1])
			return false;
		return true;
	}

	template<typename T>
	OBRect<T> get_obrect(const std::vector<Point2<T> >& points)
	{
		static Matrix<T> rot = Matrix<T>::rotate2_z(Deg2Rad(0.5));

		const T* vtx;
		double x0 = DBL_MAX, x1 = -DBL_MAX;
		double y0 = DBL_MAX, y1 = -DBL_MAX;
		for (int i = 0; i < points.size(); ++i) {
			vtx = points[i].buffer();
			if (vtx[0] < x0) x0 = vtx[0];
			if (vtx[0] > x1) x1 = vtx[0];
			if (vtx[1] < y0) y0 = vtx[1];
			if (vtx[1] > y1) y1 = vtx[1];
		}

		Point2<T> cnt((x0+x1)*0.5, (y0+y1)*0.5);

		Vector2<T> vect(1, 1);
		vect.normalize();

		double e0, e1;
		double e_min = DBL_MAX, e;
		int time = 1;
		Vector2<T> vect_best;
		double e0_best, e1_best;

		while (time < 360) {
			getProjection<T>(points, cnt, vect, e0, e1);
			e = e1 - e0;
			if (e < e_min) {
				e_min = e;
				vect_best = vect;
				e0_best = e0;
				e1_best = e1;
			}
			vect = rot * vect;
			++time;
		}

		vect[0] = vect_best[1];
		vect[1] = -vect_best[0];
		getProjection<T>(points, cnt, vect, e0, e1);

		OBRect<T> orect;
		orect._axises[0][0] = vect[0];
		orect._axises[0][1] = vect[1];
		orect._extents[0] = (e1 - e0) * 0.5;

		orect._axises[1][0] = vect_best[0];
		orect._axises[1][1] = vect_best[1];
		orect._extents[1] = (e1_best - e0_best) * 0.5;

		cnt = cnt + e0*vect + e0_best * vect_best +
			(e1 - e0) * 0.5 * vect + (e1_best - e0_best)*0.5 * vect_best;
		orect._center[0] = cnt[0];
		orect._center[1] = cnt[1];

		return orect;
	}

	template class MPCDPS_CORE_ITEM OBRect<float>;
	template class MPCDPS_CORE_ITEM OBRect<double>;

	template MPCDPS_CORE_ITEM OBRect<float> get_obrect(const std::vector<Point2<float> >& points);
	template MPCDPS_CORE_ITEM OBRect<double> get_obrect(const std::vector<Point2<double> >& points);
}