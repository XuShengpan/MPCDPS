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

#ifndef  MPCDPS_HOUGHTRANSFORM_H
#define  MPCDPS_HOUGHTRANSFORM_H

#include <vector>
#include "Point2.h"
#include "Point3.h"
#include "Line2.h"
#include "Line3.h"
#include "Plane.h"
#include "Circle.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /**\brief hough_transform_detect_line_core
     return inlier point number.

     parameters:
     points: [in] the input points
     planes: [out] the output planes
     angle_granularity_deg: [in] angle granularity in degree, default is 1
     dist_granularity: [in] distance granularity in meter, default is 0.1
     min_point: [in] minimal point number to be a valid plane, default is 100
    */
    template <typename T> MPCDPS_CORE_ITEM
    int hough_transform_detect_line_core(
        const std::vector<Point2<T> >& points,
        Line2<T>& line,
        float angle_granularity_deg = 1,
        float dist_granularity = 0.1);

    /**\brief hough_transform_detect_line
     points: [in] the input points
     planes: [out] the output planes

     angle_granularity_deg: [in] angle granularity in degree, default is 1
     dist_granularity: [in] distance granularity in meter, default is 0.1
     min_point: [in] minimal point number to be a valid plane, default is 100
    */
    template <typename T> MPCDPS_CORE_ITEM
        void hough_transform_detect_line(
            const std::vector<Point2<T> >& points,
            std::vector<Line2<T> >& lines,
            float angle_granularity_deg = 1,
            float dist_granularity = 0.1,
            int min_point = 100);

    template <typename T> MPCDPS_CORE_ITEM
        void hough_transform_detect_line(
            const std::vector<Point2<T> >& points,
            std::vector<Line2<T> >& lines,
            std::vector<std::vector<int> >& line_points,
            float angle_granularity_deg = 1,
            float dist_granularity = 0.1,
            int min_point = 20,
            float min_length = 2.0);

    /**\brief hough_transform_detect_plane_core
     return inlier point number.

     parameters:
     points: [in] the input points
     planes: [out] the output planes
     angle_granularity_deg: [in] angle granularity in degree, default is 2
     dist_granularity: [in] distance granularity in meter, default is 0.1
    */
    template <typename T> MPCDPS_CORE_ITEM
    int hough_transform_detect_plane_core(
        const std::vector<Point3<T> >& points,
        Plane<T>& plane,
        float angle_granularity_deg = 2,
        float dist_granularity = 0.1);

    /**\brief hough_transform_detect_plane
     points: [in] the input points
     planes: [out] the output planes

     angle_granularity_deg: [in] angle granularity in degree, default is 2
     dist_granularity: [in] distance granularity in meter, default is 0.1
     min_point: [in] minimal point number to be a valid plane, default is 100
    */
    template <typename T> MPCDPS_CORE_ITEM
        void hough_transform_detect_plane(
            const std::vector<Point3<T> >& points,
            std::vector<Plane<T> >& planes,
            float angle_granularity_deg = 2,
            float dist_granularity = 0.1,
            int min_point = 100);

    /**\brief hough_transform_detect_line_core
     return inlier point number.

     parameters:
     points: [in] the input points
     circle: [out] the output circle
    */
    template <typename T> MPCDPS_CORE_ITEM
        int hough_transform_detect_circle_core(
            const std::vector<Point2<T> >& points,
            Circle<T>& circle,
            float r_min = 0.1,
            float r_max = 2,
            float dist_granularity = 0.05);

}

#endif