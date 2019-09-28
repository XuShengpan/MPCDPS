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

#include "Circle.h"
#include "Line2.h"
#include "SmartArray.h"

namespace mpcdps {

    template <typename T>
    Circle<T>::Circle():_radius(0)
    {

    }

    template <typename T>
    Circle<T>::Circle(const Circle& obj):_center(obj._center), _radius(obj._radius)
    {
    }

    template <typename T>
    Circle<T>::~Circle()
    {

    }

    template <typename T>
    bool Circle<T>::intersects(const Circle<T>& obj) const
    {
        Vector2<T> v = obj._center - _center;
        return v.norm() < _radius + obj._radius;
    }

    template <typename T>
    bool Circle<T>::contains(const Circle<T>& obj) const
    {
        Vector2<T> v = obj._center - _center;
        return v.norm() + obj._radius < _radius;
    }

	template <typename T>
	bool Circle<T>::contains(const Point2<T>& pt) const
	{
		Vector2<T> v = pt - _center;
		return (v.norm() < _radius);
	}

    template <typename T>
    Circle<T>& Circle<T>::combine(const Circle<T>& obj)
    {
        if (!contains(obj)) {
            if (obj.contains(*this)) {
                _center = obj._center;
                _radius = obj._radius;
            } else {
                Point2<T> b = _center;
                Point2<T> e = obj._center;
                Vector2<T> v = e - b;
                double d = v.norm();
                v.normalize();
                b = b - _radius * v;
                e = e + obj._radius * v;
                _radius = 0.5 * (_radius + d + obj._radius);
                _center = b + _radius * v;
            }
        }
        return *this;
    }

    template <typename T>
    Circle<T> Circle<T>::combined(const Circle<T>& obj) const
    {
        if (contains(obj)) {
            return *this;
        } else if (obj.contains(*this)) {
            return obj;
        } else {
            Point2<T> b = _center;
            Point2<T> e = obj._center;
            Vector2<T> v = e - b;
            double d = v.norm();
            v.normalize();
            b = b - _radius * v;
            e = e + obj._radius * v;
            Circle<T> sph;
            sph._radius = 0.5 * (_radius + d + obj._radius);
            sph._center = b + sph._radius * v;
            return sph;
        }
    }

	template <typename T>
	bool Circle<T>::intersects(const Rect2<T>& rect) const
	{
		double r1 = rect.radius();
		Vector2<T> v = rect.center() - _center;
		double d = v.norm();
		if (d >= r1 + _radius)
			return false;
		double a = rect.length(0) * 0.5, b = rect.length(1) * 0.5;
		T ex[] = { 1, 0 };
		T ey[] = { 0, 1 };
		double a1 = v.dot(VectorK<T, 2>(ex)), b1 = v.dot(VectorK<T, 2>(ey));
		if (d < _radius + std::sqrt(a1 * a1 + b1 * b1))
			return true;
		if (std::abs(rect.min(0) - _center[0]) < _radius + a)
			return true;
		if (std::abs(rect.max(0) - _center[0]) < _radius + a)
			return true;
		if (std::abs(rect.min(1) - _center[1]) < _radius + b)
			return true;
		if (std::abs(rect.max(1) - _center[1]) < _radius + b)
			return true;
		return false;
	}

    template <typename T>
    bool Circle<T>::create(const Point2<T>& p1, const Point2<T>& p2, const Point2<T>& p3, float min_angle)
    {
        Vector2<T> v1 = p2 - p1;
        Vector2<T> v2 = p3 - p2;
        double cos_theta = std::cos(Deg2Rad(min_angle));
        if (std::abs(v1.dot(v2)) > cos_theta) {
            return false;
        }
        Vector2<T> u1(-v1[1], v1[0]);
        Vector2<T> u2(-v2[1], v2[0]);
        u1.normalize();
        u2.normalize();

        Point2<T> p((p1[0] + p2[0])*0.5, (p1[1] + p2[1])*0.5);  //p1-p2
        Point2<T> q((p2[0] + p3[0])*0.5, (p2[1] + p3[1])*0.5);  //p1-p2
        Vector2<T> pq = p - q;
        Vector2<T> u = u2 - u1;
        if (std::abs(u[0]) < 1e-2) {
            _radius = pq[1] / u[1];
        } else if (std::abs(u[1]) < 1e-2) {
            _radius = pq[0] / u[0];
        } else {
            _radius = (pq[0] / u[0] + pq[1] / u[1]) * 0.5;
        }
        _center = p + _radius * u1;

        return true;
    }

    //https://blog.csdn.net/liyuanbhu/article/details/50889951
    template <typename T>
    bool circle_fit_leastsquare(const std::vector<Point2<T> >& points, Circle<T>& circle)
    {
        double x = 0;
        double y = 0;
        const int n_point = points.size();
        for (int i = 0; i < n_point; ++i) {
            x += points[i][0];
            y += points[i][1];
        }
        x /= double(n_point);
        y /= double(n_point);
        double Suuu = 0, Svvv = 0, Suu = 0, Svv = 0, Suv = 0, Suuv = 0, Suvv = 0;
        for (int i = 0; i < n_point; ++i) {
            double u = points[i][0] - x;
            double v = points[i][1] - y;
            double uu = u * u;
            double vv = v * v;
            Suuu += uu * u;
            Svvv += vv * v;
            Suu += uu;
            Svv += vv;
            Suv += u * v;
            Suuv += uu * v;
            Suvv += u * vv;
        }
        double a = (Suuu + Suvv)/2.0;
        double b = (Suuv + Svvv) / 2.0;
        double c = Suu * Svv - Suv * Suv;
        if (std::abs(c) < ZERO_F) {
            return false;
        }

        double uc = (a * Svv - Suv * b) / c;
        double vc = (Suu * b - a * Suv) / c;
        double xc = uc + x;
        double yc = vc + y;
        double r2 = 0;
        for (int i = 0; i < n_point; ++i) {
            double dx = points[i][0] - xc;
            double dy = points[i][1] - yc;
            r2 += dx * dx + dy * dy;
        }
        double r = std::sqrt(r2);
        if (r < 0.02) {
            return false;
        }
        circle._center = Point2<T>(xc, yc);
        circle._radius = r;
        return true;
    }

    template <typename T>
    bool  circle_fit_ransac(const std::vector<Point2<T> >& points,
        Circle<T>& circle_best,
        std::vector<int>& inliers,
        double dist_error_threshold,
        int max_iter_time,
        double enough_good)
    {
        inliers.clear();
        bool found = false;
        int n = points.size();
        int iter_time = 0;

        const int n_enough_good = n * enough_good;

        std::srand(n);
        Point2<T> model[3];
        SmartArray<bool> tag(n);
        int j;
        double err;
        int select_time = 0;
        int max_select_time = max_iter_time * 2;
        while (iter_time < max_iter_time && select_time < max_select_time) {
            tag.reset(false);
            for (int k = 0; k < 3; ++k) {
                j = std::rand() % n;
                while (tag[j]) {
                    j = std::rand() % n;
                }
                model[k] = points[j];
                tag[j] = true;
            }
            ++select_time;

            Circle<T> circle;
            if (!circle.create(model[0], model[1], model[2], 10)) {
                continue;
            }

            std::vector<int> inliers_temp;
            for (j = 0; j < n; ++j) {
                Vector2<T> v = points[j] - circle._center;
                float err = std::abs(v.norm() - circle._radius);
                if (err <= dist_error_threshold) {
                    inliers_temp.push_back(j);
                }
            }

            if (inliers_temp.size() > inliers.size()) {
                inliers = inliers_temp;
                circle_best = circle;
                found = true;
                if (inliers_temp.size() >= n_enough_good) {
                    break;
                }
            }
            ++iter_time;
        }

        //         if (found) {
        //             std::vector<Point2<T> >inlier_samples;
        //             for (int i = 0; i < inliers.size(); ++i) {
        //                 inlier_samples.push_back(points[inliers[i]]);
        //             }
        //             return circle_fit_leastsquare(iner_samples, plane_bestfit);
        //         }

        return found;
    }

    //---------------------------------------------------------------------------------------------------//

    template MPCDPS_CORE_ITEM
        bool circle_fit_leastsquare(const std::vector<Point2<float> >& points, Circle<float>& circle);

    template MPCDPS_CORE_ITEM
        bool circle_fit_leastsquare(const std::vector<Point2<double> >& points, Circle<double>& circle);

    //---------------------------------------------------------------------------------------------------//

    template MPCDPS_CORE_ITEM
        bool  circle_fit_ransac(const std::vector<Point2<float> >& points,
            Circle<float>& circle,
            std::vector<int>& inliers,
            double dist_error_threshold,
            int max_iter_time,
            double enough_good
        );

    template MPCDPS_CORE_ITEM
        bool  circle_fit_ransac(const std::vector<Point2<double> >& points,
            Circle<double>& circle,
            std::vector<int>& inliers,
            double dist_error_threshold,
            int max_iter_time,
            double enough_good
        );

    //---------------------------------------------------------------------------------------------------//

    template MPCDPS_CORE_ITEM class Circle<float>;
    template MPCDPS_CORE_ITEM class Circle<double>;
}