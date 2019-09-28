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

#ifndef   MPCDPS_CIRCLE_H
#define  MPCDPS_CIRCLE_H

#include "Point2.h"
#include "Rect2.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /*class Circle.*/
    template <typename T>
    class MPCDPS_CORE_ITEM  Circle
    {
    public:
        /*Default constructor.*/
        Circle();

        /*Copy constructor.*/
        Circle(const Circle& obj);

        /*Destructor.*/
        ~Circle();

        //create a circle by 3 points.
        bool create(const Point2<T>& p1, const Point2<T>& p2, const Point2<T>& p3, float min_angle = 20);

        /*To test whether it intersects with another circle.*/
        bool intersects(const Circle<T>& obj) const;

		/*To test whether it intersects a rect.*/
		bool intersects(const Rect2<T>& rect) const;

        /*To test whether it contains another circle.*/
        bool contains(const Circle<T>& obj) const;

		/*To test whether it contains a point*/
		bool contains(const Point2<T>& pt) const;

        /*Combine with another circle and update this circle.*/
        Circle<T>& combine(const Circle<T>& obj);

        /*Combined with another circle and return a new circle.*/
        Circle<T> combined(const Circle<T>& obj) const;

        Point2<T>   _center;  /*The center of the circle.*/
        double        _radius;   /*The radius of the circle.*/
    };

    /**@  Fit a circle by least square.*/
    template <typename T> MPCDPS_CORE_ITEM
        bool circle_fit_leastsquare(const std::vector<Point2<T> >& points, Circle<T>& circle);

    /*Fit circle by RANSAC.
     *  about max_iter_time:
     *      log(0.01)/log(1-0.5^3) = 35, log(0.01)/log(1-0.4^3) = 69;
     *      we assume that most of the samples are inners.
     *      so set max_iter_time = 50
     *  If enough_good percent of the points fit the model, the algorithm will return.
     *
     *  \param  inners: the ids in points.
     */
    template <typename T> MPCDPS_CORE_ITEM
        bool  circle_fit_ransac(const std::vector<Point2<T> >& points,
            Circle<T>& circle,
            std::vector<int>& inliers,
            double dist_error_threshold = 0.1,
            int max_iter_time = 50,
            double enough_good = 0.8
        );
}

#endif