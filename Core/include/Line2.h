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

#ifndef   MPCDPS_LINE2_H
#define  MPCDPS_LINE2_H

#include "Point2.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /*class Plane.
    *  A Line2 exists in XOY plane and is defined as 
    *  ax + by = c
    *  The directional vector is V(b, -a).
    *  Note:  (a, b) is a unit vector.
    */
    template<typename T>
    class MPCDPS_CORE_ITEM Line2
    {
    public:
        /* Constructor.*/
        Line2();

        /* Constructor: construct a Line2 with two points. */
        Line2(const Point2<T>& p0, const Point2<T>& p1);

        /* Constructor*/
        Line2(const Vector2<T>& dir, const Point2<T>& point);

        /* Get the direction of the line.*/
        Vector2<T> direction() const;

        /* Get distance of the point to the line segment.
        * IF the point lies on left side of the line segment, return  a positive distance;
        * if the point lies on right side of the line segment, return a negative distance.
        */
        double pointDistance(const Point2<T>& point) const;

        T _a;
        T _b;
        T _c;
    };

    typedef Line2<double> Line2d;
    typedef Line2<float>  Line2f;

    /**@  Fit a Line2 with least square.*/
    template <typename T> MPCDPS_CORE_ITEM
    bool line2_fit_leastsquare(const std::vector<Point2<T> >& points, Line2<T>& line);

    /**@  Fit a Line2 with least square.*/
    template <typename T> MPCDPS_CORE_ITEM
        bool line2_fit_leastsquare_w(const std::vector<Point2<T> >& pointList,
            const std::vector<double>& w, Line2<T>& line);

    /*Fit a Line2 by ransac.
    *  about max_iter_time:
    *      log(0.01)/log(1-0.5^2) = 16, log(0.01)/log(1-0.4^2) = 26;
    *      we assume that most of the samples are inners.
    *      so set max_iter_time = 25
    *  If enough_good percent of the points fit the model, the algorithm will return.
    */
    template <typename T> MPCDPS_CORE_ITEM
    bool line2_fit_ransac(const std::vector<Point2<T> >& points,
        Line2<T>& line2_bestfit,
        std::vector<Point2<T> >& iner_samples,
        double dist_error_threshold = 0.1,
        int max_iter_time = 25,
        double enough_good = 0.8
    );
}

#endif