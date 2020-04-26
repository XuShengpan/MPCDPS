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

#ifndef   MPCDPS_SPHERE_H
#define  MPCDPS_SPHERE_H

#include "Point3.h"
#include "Box3.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /*class Sphere.*/
    template <typename T>
    class MPCDPS_CORE_ITEM  Sphere
    {
    public:
        /*Default constructor.*/
        Sphere();

        /*Copy constructor.*/
        Sphere(const Sphere<T>& rth);

        /*Destructor.*/
        ~Sphere();

        /*To test whether it intersects with another sphere.*/
        bool intersects(const Sphere<T>& obj) const;

		/*To test whether it intersects with a box.*/
		bool intersects(const Box3<T>& box) const;

        /*To test whether it contains another sphere.*/
        bool contains(const Sphere<T>& obj) const;

        /*Combine with another sphere and update this sphere.*/
        Sphere<T>& combine(const Sphere<T>& obj);

        /*Combine with another sphere and return a new sphere.*/
        Sphere<T> combined(const Sphere<T>& obj) const;

        Point3<T>   _center;  /*The center of the sphere.*/
        double        _radius;   /*The radius of the sphere.*/
    };
}

#endif