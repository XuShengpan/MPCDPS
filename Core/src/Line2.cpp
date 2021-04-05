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

#include "Line2.h"
#include "Matrix.h"
#include <iostream>

namespace mpcdps {

    template<typename T>
    Line2<T>::Line2():_a(0), _b(0), _c(0)
    {

    }

    template<typename T>
    Line2<T>::Line2(const Vector2<T>& dir, const Point2<T>& point)
    {
        Vector2<T> n(-dir[1], dir[0]);
        n.normalize();
        _a = n[0];
        _b = n[1];
        _c = n.dot(Vector2<T>(point.buffer()));
    }

    template<typename T>
    Line2<T>::Line2(const Point2<T>& p0, const Point2<T>& p1)
		:Line2<T>(p1-p0, p0)
    {
    }

    template<typename T>
    bool Line2<T>::intersects(const Line2<T>& line, Point2<T>& intersection) const
    {
        T t = _a * line._b - _b * line._a;
        if (std::abs(t) < 1e-6) {
            return false;
        }
        intersection[0] = (_c * line._b - _b * line._c) / t;
        intersection[1] = (_a * line._c - _c * line._a) / t;
        return true;
    }

    template<typename T>
    Vector2<T> Line2<T>::direction() const
    {
        return Vector2<T>(-_b, _a);
    }

    template<typename T>
    double Line2<T>::pointDistance(const Point2<T>& point) const
    {
        return (_a * point[0] + _b * point[1] - _c);
    }

    bool get_max_eigen_for_matrix2(const Matrix<double>& cov2x2, double& e, Vector2d& v)
    {
        double a = cov2x2[0][0], b = cov2x2[0][1], c = cov2x2[1][0], d = cov2x2[1][1];
        double delta = (a + d) * (a + d) - 4.0 * (a * d - b * c);

        if (delta < 1e-6) {  //<= 0
            return false;
        }

        double e1 = ((a + d) + std::sqrt(delta)) / 2.0;
        //double e2 = ((a + d) - std::sqrt(delta)) / 2.0;
        e = e1;

        if (std::abs(b) < 1e-6) {
            v[0] = 0;
            v[1] = 1;
        } else {
            v[0] = 1;
            v[1] = (e - a) / b;
            v.normalize();
        }

        return true;
    }

    template<typename T>
    bool line2_fit_leastsquare(const std::vector<Point2<T> >& points, Line2<T>& line)
    {
        double x0 = 0;
        double y0 = 0;
        for (int i = 0; i < points.size(); ++i) {
            x0 += points[i][0];
            y0 += points[i][1];
        }

        x0 /= double(points.size());
        y0 /= double(points.size());

        double xx = 0;
        double xy = 0;
        double yy = 0;

        double dx = 0;
        double dy = 0;
        for (int i = 0; i < points.size(); ++i) {
            dx = points[i][0] - x0;
            dy = points[i][1] - y0;

            xx += dx * dx;
            xy += dx * dy;
            yy += dy * dy;
        }

        Matrix<double> mat_a(2, 2);
        mat_a[0][0] = xx;
        mat_a[0][1] = xy;
        mat_a[1][0] = xy;
        mat_a[1][1] = yy;

        double e;
        Vector2<double> v;
        if (!get_max_eigen_for_matrix2(mat_a, e, v)) {
            return false;
        }

        Vector2d dir(v[0], v[1]);
        dir.normalize();
        line._a = dir[1];
        line._b = -dir[0];
        line._c = dir[1] * x0 - dir[0] * y0;
        return true;
    }

    template<typename T>
    bool line2_fit_leastsquare_w(const std::vector<Point2<T> >& points,
        const std::vector<double>& weights, Line2<T>& line)
    {
        double x0 = 0;
        double y0 = 0;
        double wi = 0;
        double sw = 0;
        for (int i = 0; i < points.size(); ++i) {
            wi = weights[i];
            x0 += points[i][0] * wi;
            y0 += points[i][1] * wi;
            sw += wi;
        }

        x0 = x0 / sw;
        y0 = y0 / sw;

        double xx = 0;
        double xy = 0;
        double yy = 0;

        double dx = 0;
        double dy = 0;
        for (int i = 0; i < points.size(); ++i) {
            wi = weights[i];
            dx = points[i][0] - x0;
            dy = points[i][1] - y0;

            xx += dx * dx * wi;
            xy += dx * dy * wi;
            yy += dy * dy * wi;
        }

        Matrix<double> mat_a(2, 2);
        mat_a[0][0] = xx;
        mat_a[0][1] = xy;
        mat_a[1][0] = xy;
        mat_a[1][1] = yy;

        double e;
        Vector2<double> v;
        if (!get_max_eigen_for_matrix2(mat_a, e, v)) {
            return false;
        }

        Vector2d dir(v[0], v[1]);
        dir.normalize();
        line._a = dir[1];
        line._b = -dir[0];
        line._c = dir[1] * x0 - dir[0] * y0;
        return true;
    }

    template <typename T>
    bool line2_fit_ransac(const std::vector<Point2<T> >& pointList,
        Line2<T>& line2_bestfit,
        std::vector<int >& inliers,
        double dist_error, int max_iter_time, double enough_good)
    {
        double xmin = DBL_MAX;
        double xmax = -DBL_MAX;
        double ymin = DBL_MAX;
        double ymax = -DBL_MAX;

        for (auto pt : pointList) {
            if (pt.x() < xmin) xmin = pt.x();
            if (pt.x() > xmax) xmax = pt.x();
            if (pt.y() < ymin) ymin = pt.y();
            if (pt.y() > ymax) ymax = pt.y();
        }

        float radius = 0;
        {
            double dx = xmax - xmin;
            double dy = ymax - ymin;
            radius = std::sqrt(dx * dx + dy * dy) * 0.5;
        }

        inliers.clear();
        bool found = false;
        int n = pointList.size();
        int iter_time = 0;

        const int n_enough_good = n * enough_good;
        const float min_length = radius * 0.6;

        std::srand(n);
        Point2<T> model[2];

        double err;
        while (iter_time < max_iter_time) {
            std::vector<bool> tag(pointList.size(), false);
            std::vector<int> inliers_temp;

            int j = 0;
            int k_seed = 0;
            while (k_seed < 2) {
                j = std::rand() % n;
                if (tag[j])  continue;
                model[k_seed++] = pointList[j];
                tag[j] = true;

                if (k_seed >= 2) {
                    Vector2<T> dp = model[1] - model[0];
                    if (dp.norm() < min_length) {
                        --k_seed;
                    }
                }
            }

            Line2<T> line(model[0], model[1]);
            for (j = 0; j < n; ++j) {
                err = std::abs(line.pointDistance(pointList[j]));
                if (err <= dist_error) {
                    inliers_temp.push_back(j);
                }
            }

            if (inliers_temp.size() > inliers.size()) {
                inliers = inliers_temp;
                line2_bestfit = line;
                found = true;
                if (inliers.size() >= n_enough_good) {
                    break;
                }
            }
            ++iter_time;
        }

//         if (found) {
//             std::vector<Point2<T> > inlier_samples;
//             for (int i = 0; i < inliers.size(); ++i) {
//                 inlier_samples.push_back(pointList[inliers[i]]);
//             }
//             return line2_fit_leastsquare(inlier_samples, line2_bestfit);
//         }

        return found;
    }

    template MPCDPS_CORE_ITEM class Line2<float>;
    template MPCDPS_CORE_ITEM class Line2<double>;

    template MPCDPS_CORE_ITEM bool line2_fit_leastsquare(const std::vector<Point2<float> >& points, Line2<float>& line);
    template MPCDPS_CORE_ITEM bool line2_fit_leastsquare(const std::vector<Point2<double> >& points, Line2<double>& line);

    template MPCDPS_CORE_ITEM
        bool line2_fit_leastsquare_w(const std::vector<Point2<float> >& pointList,
            const std::vector<double>& w, Line2<float>& line);

    template MPCDPS_CORE_ITEM
        bool line2_fit_leastsquare_w(const std::vector<Point2<double> >& pointList,
            const std::vector<double>& w, Line2<double>& line);

    template MPCDPS_CORE_ITEM 
        bool line2_fit_ransac(const std::vector<Point2<float> >& points,
        Line2<float>& line2_bestfit,
        std::vector<int >& inliers,
        double dist_error_threshold,
        int max_iter_time,
        double enough_good
    );

    template MPCDPS_CORE_ITEM
        bool line2_fit_ransac(const std::vector<Point2<double> >& points,
        Line2<double>& line2_bestfit,
        std::vector<int >& inliers,
        double dist_error_threshold,
        int max_iter_time,
        double enough_good
    );

}