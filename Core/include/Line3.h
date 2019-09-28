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

#ifndef  MPCDPS_LINE3_H
#define  MPCDPS_LINE3_H

#include "Point3.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /**@brief class Plane.
                *  A line3 is defined by a point and a direction.
                *  X = X0 + t*Dir.
                */
    template<typename T>
    class MPCDPS_CORE_ITEM  Line3
    {
    public:
        /**@brief  Constructor.*/
        Line3();

        /**@brief  Constructor: construct a Line2 with two points. */
        Line3(const Point3<T>& p0, const Point3<T>& p1);

        /**@brief  Constructor*/
        Line3(const Vector3<T>& dir, const Point3<T>& point);

        /**@brief  Get the direction of the line.*/
        Vector3<T> direction() const;

        /**@brief  Get distance of the point to the line segment.
        * Distance to a line3 is larger than or equal with 0.
        */
        double pointDistance(const Point3<T>& point) const;

        Point3<T> _point;
        Vector3<T> _dir;
    };

    typedef Line3<double> Line3d;
    typedef Line3<float>    Line3f;

    /**@  Fit a Line3 with least square.*/
    template <typename T> 
        bool line3_fit_leastsquare(const std::vector<Point3<T> >& points, Line3<T>& line);

    template <typename T> 
        bool line3_fit_leastsquare_w(const std::vector<Point3<T> >& points,
            const std::vector<double>& weight, Line3<T>& line);

    /**@brief Fit a Line3 by ransac.
    *  about max_iter_time:
    *      log(0.01)/log(1-0.5^2) = 16, log(0.01)/log(1-0.4^2) = 26;
    *      we assume that most of the samples are inners.
    *      so set max_iter_time = 25
    *  If enough_good percent of the points fit the model, the algorithm will return.
    */
    template <typename T> 
        bool line3_fit_ransac(const std::vector<Point3<T> >& points,
            Line3<T>& line3_bestfit,
            std::vector<int>& inliers,
            double dist_error_threshold = 0.1,
            int max_iter_time = 25,
            double enough_good = 0.8
        );
}

#endif