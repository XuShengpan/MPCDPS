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

#include "Line3.h"
#include "Matrix.h"

namespace mpcdps {

    template<typename T>
    Line3<T>::Line3()
    {

    }

    template<typename T>
    Line3<T>::Line3(const Point3<T>& p0, const Point3<T>& p1)
        :_point(p0)
    {
        _dir = p1 - p0;
        _dir.normalize();
    }

    template<typename T>
    Line3<T>::Line3(const Vector3<T>& dir, const Point3<T>& point)
        :_point(point), _dir(dir)
    {

    }

    template<typename T>
    Vector3<T> Line3<T>::direction() const
    {
        return _dir;
    }

    template<typename T>
    double Line3<T>::pointDistance(const Point3<T>& point) const
    {
        Vector3<T> v1 = point - _point;
        Vector3<T> v2 = v1 - v1.dot(_dir)*_dir;
        return v2.norm();
    }

    template<typename T>
    bool line3_fit_leastsquare(const std::vector<Point3<T> >& pointList,
        Line3<T>& line)
    {
        double x0 = 0;
        double y0 = 0;
        double z0 = 0;
        int n_points = pointList.size();
        double t = 0;
        const T* vtx = NULL;
        for (int i = 0; i < n_points; ++i) {
            t = 1.0 / double(i + 1);
            vtx = pointList[i].buffer();
            x0 = x0 * t * double(i) + vtx[0] * t;
            y0 = y0 * t * double(i) + vtx[1] * t;
            z0 = z0 * t * double(i) + vtx[2] * t;
        }

        double d2 = 0;
        double x = 0;
        double y = 0;

        double di = 0;
        for (int i = 0; i < n_points; ++i) {
            vtx = pointList[i].buffer();
            di = vtx[2] - z0;
            d2 += di * di;
            x += (vtx[0] - x0) * di;
            y += (vtx[1] - y0) * di;
        }
        if (std::abs(d2) < ZERO_F) {
            return false;
        }

        double a1 = x / d2;
        double b1 = y / d2;

        double c2 = 1.0 / (1.0 + a1 * a1 + b1 * b1);
        double a2 = c2 * a1 * a1;
        double b2 = c2 * b1 * b1;

        double a = std::sqrt(a2);
        double b = std::sqrt(b2);
        double c = std::sqrt(c2);
        if (a1 < 0) {
            a = -a;
        }
        if (b1 < 0) {
            b = -b;
        }

        line._point[0] = x0;
        line._point[1] = y0;
        line._point[2] = z0;
        line._dir[0] = a;
        line._dir[1] = b;
        line._dir[2] = c;

        return true;
    }

    template <typename T>
    bool line3_fit_leastsquare_w(const std::vector<Point3<T> >& points,
        const std::vector<double>& w, Line3<T>& line)
    {
        double x0 = 0;
        double y0 = 0;
        double z0 = 0;
        int n_points = points.size();
        double t = 0;
        const T* vtx = NULL;
        for (int i = 0; i < n_points; ++i) {
            t = 1.0 / double(i + 1);
            vtx = points[i].buffer();
            x0 = x0 * t * double(i) + vtx[0] * t;
            y0 = y0 * t * double(i) + vtx[1] * t;
            z0 = z0 * t * double(i) + vtx[2] * t;
        }

        double wd2 = 0;
        double xw = 0;
        double yw = 0;

        double di = 0;
        for (int i = 0; i < n_points; ++i) {
            vtx = points[i].buffer();
            di = vtx[2] - z0;
            wd2 += w[i] * di * di;
            xw += w[i] * (vtx[0] - x0) * di;
            yw += w[i] * (vtx[1] - y0) * di;
        }
        if (std::abs(wd2) < ZERO_F) {
            return false;
        }

        double a1 = xw / wd2;
        double b1 = yw / wd2;

        double c2 = 1.0 / (1.0 + a1 * a1 + b1 * b1);
        double a2 = c2 * a1 * a1;
        double b2 = c2 * b1 * b1;

        double a = std::sqrt(a2);
        double b = std::sqrt(b2);
        double c = std::sqrt(c2);
        if (a1 < 0) {
            a = -a;
        }
        if (b1 < 0) {
            b = -b;
        }

        line._point[0] = x0;
        line._point[1] = y0;
        line._point[2] = z0;
        line._dir[0] = a;
        line._dir[1] = b;
        line._dir[2] = c;

        return true;
    }

    template<typename T>
    bool line3_fit_ransac(const std::vector<Point3<T> >& pointList,
        Line3<T>& line3_bestfit,
        std::vector<int>& inliers,
        double dist_error,
        int max_iter_time,
        double enough_good
    )
    {
        inliers.clear();
        bool found = false;
        int n = pointList.size();
        int iter_time = 0;

        const int n_enough_good = n * enough_good;

        std::srand(n);
        Point3<T> model[3];
        SmartArray<bool> tag(n);
        int j;
        double err;
        while (iter_time < max_iter_time) {
            tag.reset(false);
            std::vector<int> inliers_temp;
            for (int k = 0; k < 2; ++k) {
                j = std::rand() % n;
                while (tag[j]) {
                    j = std::rand() % n;
                }
                model[k] = pointList[j];
                tag[j] = true;
            }

            if (std::abs(model[0][0] - model[1][0]) < 0.01 &&
                std::abs(model[0][1] - model[1][1]) < 0.01 &&
                std::abs(model[0][2] - model[1][2]) < 0.01)
            {
                continue;
            }

            Line3<T> fit(model[0], model[1]);
            for (j = 0; j < n; ++j) {
                err = std::abs(fit.pointDistance(pointList[j]));
                if (err <= dist_error) {
                    inliers_temp.push_back(j);
                }
            }

            if (inliers_temp.size() > inliers.size()) {
                inliers = inliers_temp;
                line3_bestfit = fit;
                found = true;
                if (inliers.size() >= n_enough_good) {
                    break;
                }
            }
            ++iter_time;
        }

//         if (found) {
//             std::vector<Point3<T> > inlier_samples;
//             for (int i = 0; i < inliers.size(); ++i) {
//                 inlier_samples.push_back(pointList[inliers[i]]);
//             }
//             return line3_fit_leastsquare(inlier_samples, line3_bestfit);
//         }

        return found;
    }

    template MPCDPS_CORE_ITEM class Line3<float>;
    template MPCDPS_CORE_ITEM class Line3<double>;

    template MPCDPS_CORE_ITEM bool line3_fit_leastsquare(
        const std::vector<Point3<float> >& pointList,
        Line3<float>& line);
    template MPCDPS_CORE_ITEM bool line3_fit_leastsquare(
        const std::vector<Point3<double> >& pointList,
        Line3<double>& line);

    template MPCDPS_CORE_ITEM bool line3_fit_ransac(
        const std::vector<Point3<float> >& pointList,
        Line3<float>& line3_bestfit,
        std::vector<int>& inliers,
        double dist_error,
        int max_iter_time,
        double enough_good
    );

    template MPCDPS_CORE_ITEM bool line3_fit_ransac(
        const std::vector<Point3<double> >& pointList,
        Line3<double>& line3_bestfit,
        std::vector<int>& inliers,
        double dist_error,
        int max_iter_time,
        double enough_good
    );

    template MPCDPS_CORE_ITEM bool line3_fit_leastsquare_w(
        const std::vector<Point3<float> >& pointList,
        const std::vector<double>& weight,
        Line3<float>& line);
    template MPCDPS_CORE_ITEM bool line3_fit_leastsquare_w(
        const std::vector<Point3<double> >& pointList,
        const std::vector<double>& weight,
        Line3<double>& line);
}