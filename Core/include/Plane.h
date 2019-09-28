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

#ifndef  MPCDPS_PLANE_H
#define MPCDPS_PLANE_H

#include "Point3.h"
#include "LineSegment3.h"
#include "VectorK.h"

namespace mpcdps {

    /*class Plane.
     *  A plane exists in 3D space and is defined as NX = c where as N is the normal and c is a constant.
     */
    template <typename T>
    class MPCDPS_CORE_ITEM  Plane
    {
    public:

        /*Default constructor.*/
        Plane();

        /*Constructor: construct a plane with the normal and a point that lies on the plane.*/
        Plane(const Vector3<T>& normal, const Point3<T>& pt);

        /*Create a plane with three points, return true if success. */
        bool create(const Point3<T>& pt1, const Point3<T>& pt2, const Point3<T>& pt3); 

        /*Compute the distance of the point to the plane.
         * If the point lies in the left side the normal points to, return a positive distance;
         * If the point lies in the back side the normal points to, return a negative distance.
         * If the point lies on the plane, return 0;
         */
        double pointDistance(const Point3<T>& pt) const;

        /*To test whether the line intersects with the plane.
         *  If intersects, return true and save the intersection; 
         *  otherwise return false and the intersection is undefined.
         */
        bool intersects_line(const LineSegment3<T>& line, Point3<T>& intersection) const;

        /*To test whether the ray intersects with the plane.
        *  If intersects, return true and save the intersection;
        *  otherwise return false and the intersection is undefined.
        */
        bool intersects_ray(const Point3<T>& ray_start, const Vector3<T>& ray_dir, 
            Point3<T>& intersection) const;

        bool interpolate_z(double x, double y, double& z) const;

        Vector3<T> _normal;  /*The normal of the plane.*/
        double _c;    /*The constant of the plane.*/
    };

    template <typename T> MPCDPS_CORE_ITEM
    bool  plane_fit_leastsquare(const std::vector<Point3<T> >& points, Plane<T>& plane);

    template <typename T> MPCDPS_CORE_ITEM
    bool  plane_fit_leastsquare_w(const std::vector<Point3<T> >& points, 
        const std::vector<double>& weights, Plane<T>& plane);

    /*Fit plane by ransac.
     *  about max_iter_time:
     *      log(0.01)/log(1-0.5^3) = 35, log(0.01)/log(1-0.4^3) = 69;
     *      we assume that most of the samples are inners.
     *      so set max_iter_time = 50
     *  If enough_good percent of the points fit the model, the algorithm will return.
     *
     *  \param  inners: the ids in points.
     */
    template <typename T> MPCDPS_CORE_ITEM
    bool  plane_fit_ransac(const std::vector<Point3<T> >& points,
        Plane<T>& plane_bestfit,
        std::vector<int>& inliers,
        double dist_error_threshold = 0.1,
        int max_iter_time = 50,
        double enough_good = 0.8
    );

    template<typename T> MPCDPS_CORE_ITEM
    bool plane_fit_best(const std::vector<Point3<T> >& points,
        mpcdps::Plane<T>& plane, float inner_dist = 0.25);

}

#endif