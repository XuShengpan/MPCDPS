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

#ifndef  MPCDPS_OBRECT_H
#define MPCDPS_OBRECT_H

#include "Point2.h"
#include "VectorK.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps
{
	template <typename T>
	class MPCDPS_CORE_ITEM OBRect 
	{
	public:
		OBRect();
		~OBRect();

		T getRadius() const;
		bool contains(const Point2<T>& point) const;

		void getVertices(Point2<T> vertices[4]) const;

		Point2<T>  _center;
		Vector2<T> _axises[2];
		T  _extents[2];
	};

	typedef OBRect<float> OBRectf;
	typedef OBRect<double> OBRectd;

    //get a flat OBRect.
	template <typename T> MPCDPS_CORE_ITEM
	OBRect<T> get_obrect(const std::vector<Point2<T> >& points);

}

#endif