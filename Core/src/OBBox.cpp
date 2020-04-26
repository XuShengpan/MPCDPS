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

#include "OBBox.h"
#include "Point2.h"
#include "VectorK.h"
#include "Matrix.h"

namespace mpcdps {

	template<typename T>
	OBBox<T>::OBBox()
	{

	}

	template<typename T>
	OBBox<T>::~OBBox()
	{

	}

	/*
	axis2(-):   0 1 2 3
	axis2(+):  4 5 6 7

	            axis0(-)  axis0(+)
	axis1(-)     0,4       1,5
	axis1(+)    2,6       3,7
	*/
	template<typename T>
	void OBBox<T>::getVertices(Point3<T> vertices[8]) const
	{
		Vector3<T> axis0 = _extents[0] * _axises[0];
		Vector3<T> axis1 = _extents[1] * _axises[1];
		Vector3<T> axis2 = _extents[2] * _axises[2];
		vertices[0] = _center - axis0 - axis1 - axis2;
		vertices[1] = _center + axis0 - axis1 - axis2;
		vertices[2] = _center + axis0 + axis1 - axis2;
		vertices[3] = _center - axis0 + axis1 - axis2;
		vertices[4] = _center - axis0 - axis1 + axis2;
		vertices[5] = _center + axis0 - axis1 + axis2;
		vertices[6] = _center + axis0 + axis1 + axis2;
		vertices[7] = _center - axis0 + axis1 + axis2;
	}

	template<typename T>
	T OBBox<T>::getRadius() const
	{
		return std::sqrt(_extents[0] * _extents[0] + _extents[1] * _extents[1] + _extents[2] * _extents[2]);
	}

	template<typename T>
	bool OBBox<T>::contains(const Point3<T>& point) const
	{
		T e[3];
		Vector3<T> op = point - _center;
		e[0] = op.dot(_axises[0]);
		if (std::abs(e[0]) > _extents[0])
			return false;
		e[1] = op.dot(_axises[1]);
		if (std::abs(e[1]) > _extents[1])
			return false;
		e[2] = op.dot(_axises[2]);
		if (std::abs(e[2]) > _extents[2])
			return false;
		return true;
	}

	template<typename T>
	void getProjection(const std::vector<Point3<T> >& points, const Point2d& cnt, 
		const Vector2d& axis, double& e0, double & e1)
	{
		Vector2d v;
		e0 = DBL_MAX;
		e1 = -DBL_MAX;
		double e;
		for (int i = 0; i < points.size(); ++i) {
			v = Point2d(points[i][0], points[i][1]) - cnt;
			e = v.dot(axis);
			if (e < e0) e0 = e;
			if (e > e1) e1 = e;
		}
	}

	template<typename T>
	OBBox<T> get_obbox(const std::vector<Point3<T> >& points)
	{
		static Matrix<double> _rot = Matrix<double>::rotate2_z(Deg2Rad(0.5));

		Point3<T> vtx;
		double z0 = DBL_MAX, z1 = -DBL_MAX;
		double x0 = DBL_MAX, x1 = -DBL_MAX;
		double y0 = DBL_MAX, y1 = -DBL_MAX;
		for (size_t i = 0; i < points.size(); ++i) {
			vtx = points[i];
			if (vtx[0] < x0) x0 = vtx[0];
			if (vtx[0] > x1) x1 = vtx[0];
			if (vtx[1] < y0) y0 = vtx[1];
			if (vtx[1] > y1) y1 = vtx[1];
			if (vtx[2] < z0) z0 = vtx[2];
			if (vtx[2] > z1) z1 = vtx[2];
		}

		Vector2d vect(1, 1);
		vect.normalize();

		double e0, e1;
		double e_min = DBL_MAX, e;
		int time = 1;
		Vector2d vect_best;
		Point2d cnt2d((x0 + x1)*0.5, (y0 + y1)*0.5);
		double e0_best, e1_best;

		while (time < 360) {
			getProjection(points, cnt2d, vect, e0, e1);
			e = e1 - e0;
			if (e < e_min) {
				e_min = e;
				vect_best = vect;
				e0_best = e0;
				e1_best = e1;
			}
			vect = _rot * vect;
			++time;
		}

		vect[0] = vect_best[1];
		vect[1] = -vect_best[0];
		getProjection(points, cnt2d, vect, e0, e1);

		OBBox<T> box;
		box._axises[0][0] = vect[0];
		box._axises[0][1] = vect[1];
		box._axises[0][2] = 0;
		box._extents[0] = (e1 - e0) * 0.5;

		box._axises[1][0] = vect_best[0];
		box._axises[1][1] = vect_best[1];
		box._axises[1][2] = 0;
		box._extents[1] = (e1_best - e0_best) * 0.5;

		box._axises[2][0] = 0;
		box._axises[2][1] = 0;
		box._axises[2][2] = 1;
		box._extents[2] = (z1 - z0) * 0.5;

		Point2d cnt = cnt2d + e0*vect + e0_best * vect_best +
			(e1 - e0)*0.5*vect + (e1_best - e0_best)*0.5 * vect_best;
		box._center[0] = cnt[0];
		box._center[1] = cnt[1];
		box._center[2] = (z1 + z0) * 0.5;

		return box;
	}

	template MPCDPS_CORE_ITEM class OBBox<float>;
	template MPCDPS_CORE_ITEM class OBBox<double>;

	template MPCDPS_CORE_ITEM OBBox<float> get_obbox(const std::vector<Point3<float> >& points);
	template MPCDPS_CORE_ITEM OBBox<double> get_obbox(const std::vector<Point3<double> >& points);

}