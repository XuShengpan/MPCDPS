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

#ifndef  MPCDPS_OBBOX_H
#define MPCDPS_OBBOX_H

#include "Point3.h"
#include "VectorK.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

	template <typename T>
	class MPCDPS_CORE_ITEM OBBox
	{
	public:
		OBBox();
		~OBBox();

		/* 
		 axis2(-):   0 1 2 3
		 axis2(+):  4 5 6 7

					axis0(-)  axis0(+)
		 axis1(-)     0,4       1,5
		 axis1(+)    2,6       3,7
		*/
		void getVertices(Point3<T> vertices[8]) const;
		T getRadius() const;

		bool contains(const Point3<T>& point) const;

		Point3<T>  _center;
		Vector3<T> _axises[3];
		T _extents[3];  // Vi: e[i] > 0
	};

    //Get a flat OBBox.
	template <typename T>
	OBBox<T> get_obbox(const std::vector<Point3<T> >& points);

}

#endif