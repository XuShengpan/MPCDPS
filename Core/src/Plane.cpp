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

#include "Plane.h"
#include "Matrix.h"
#include <iostream>

namespace mpcdps {

    template <typename T>
    Plane<T>::Plane()
    {

    }

    template <typename T>
    Plane<T>::Plane(const Vector3<T>& n, const Point3<T>& pt)
    {
        _normal = n;
        _normal.normalize();
        _c = _normal.dot(Vector3<T>(pt.buffer()));
    }

    template <typename T>
    bool Plane<T>::create(const Point3<T>& pt1, const Point3<T>& pt2, 
        const Point3<T>& pt3)
    {
        Vector3<T> v1 = pt2 - pt1;
        if (v1.norm() < ZERO_F) {
            return false;
        }
        Vector3<T> v2 = pt3 - pt1;
        if (v2.norm() < ZERO_F) {
            return false;
        }
        Vector3<T> v = v1.cross(v2);
        if (v.norm() < ZERO_F) {
            return false;
        }

        _normal = v;
        _normal.normalize();
        _c = _normal.dot(Vector3<T>(pt1.buffer()));
        return true;
    }

    template <typename T>
    double Plane<T>::pointDistance(const Point3<T>& pt) const
    {
        return Vector3<T>(pt.buffer()).dot(_normal) - _c;
    }

    template <typename T>
    bool Plane<T>::intersects_line(const LineSegment3<T>& line, 
        Point3<T>& intersection) const
    {
        Vector3<T> dir = line._end - line._start;
        dir.normalize();
        double det = dir.dot(_normal);
        if (std::abs(det) < ZERO_F) {
            return false;
        }

        double h1 = pointDistance(line._start);
        double h2 = pointDistance(line._end);
        if (std::abs(h1) < ZERO_F) {
            intersection = line._start;
            return true;
        } else if (std::abs(h2) < ZERO_F) {
            intersection = line._end;
            return true;
        }

        if (h1 * h2 > 0) {
            return false;
        }

        double d = std::abs(h1 / det);
        intersection = line._start + d * dir;
        return true;
    }

    template <typename T>
    bool Plane<T>::intersects_ray(const Point3<T>& ray_start, 
        const Vector3<T>& ray_dir, Point3<T>& intersection) const
    {
        double nd = _normal.dot(ray_dir);
        if (std::abs(nd) < ZERO_F) {
            return false;
        }
        double t = (_c - _normal.dot(ray_start.buffer())) / nd;
        if (std::abs(t) < ZERO_F) {
            intersection = ray_start;
            return true;
        }
        if (t < ZERO_F) {
            return false;
        }
        intersection = ray_start + t * ray_dir;
        return true;
    }

    template <typename T>
    bool Plane<T>::interpolate_z(double x, double y, double& z) const
    {
        if (std::abs(_normal[2]) > 1e-6) {
            z = (_c - _normal[0] * x - _normal[1] * y) / _normal[2];
            return true;
        } else {
            return false;
        }
    }

    template <typename T>
    bool plane_fit_leastsquare(const std::vector<Point3<T> >& points, Plane<T>& plane)
    {
        if (points.size() < 3) {
            return false;
        }

        const size_t n = points.size();
        const T* vtx = NULL;
        double x0 = 0, y0 = 0, z0 = 0;
        double t;
        for (size_t i = 1; i < n; ++i) {
            vtx = points[i].buffer();
            t = 1.0 / double(i + 1);
            x0 = x0 * t * double(i) + vtx[0] * t;
            y0 = y0 * t * double(i) + vtx[1] * t;
            z0 = z0 * t * double(i) + vtx[2] * t;
        }

        double dx, dy, dz;
        double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        for (size_t i = 0; i < n; ++i) {
            vtx = points[i].buffer();
            dx = vtx[0] - x0;
            dy = vtx[1] - y0;
            dz = vtx[2] - z0;
            xx += dx * dx;
            xy += dx * dy;
            xz += dx * dz;
            yy += dy * dy;
            yz += dy * dz;
            zz += dz * dz;
        }
        double data[] = {
            xx, xy, xz,
            xy, yy, yz,
            xz, yz, zz
        };

        Matrix<double> cov(3, 3, data);
        Matrix<double> cov1 = cov.clone();
        int rk = cov1.rank();
        if (rk == 3) {
            double e;
            Matrix<double> ev;
            cov.eigenMin(e, ev);
            Vector3d ev1(ev.buffer());
            plane._normal = Vector3<T>(ev1[0], ev1[1], ev1[2]);
            plane._c = ev1.dot(Vector3d(x0, y0, z0));
        } else {
            if (rk == 2) {
                if (std::abs(cov1[1][1] - 1.0) < 0.001) {
                    plane._normal[0] = cov1[0][2];
                    plane._normal[1] = cov1[1][2];
                    plane._normal[2] = -1;
                    plane._normal.normalize();
                    plane._c = plane._normal.dot(Vector3<T>(x0, y0, z0));
                } else if (std::abs(cov1[0][0] - 1.0) < 0.001) {
                    plane._normal[0] = cov1[0][1];
                    plane._normal[1] = -1;
                    plane._normal[2] = 0;
                    plane._normal.normalize();
                    plane._c = plane._normal.dot(Vector3<T>(x0, y0, z0));
                } else {
                    plane._normal[0] = 1;
                    plane._normal[1] = 0;
                    plane._normal[2] = 0;
                    plane._normal.normalize();
                    plane._c = plane._normal.dot(Vector3<T>(x0, y0, z0));
                }
            } else {
                //The plane can't be defined.
                return false;
            }
        }
        return true;
    }

    template <typename T>
    bool plane_fit_leastsquare_w(const std::vector<Point3<T> >& points,
        const std::vector<double>& weights, Plane<T>& plane)
    {
        if (points.size() < 3) {
            return false;
        }

        const size_t n = points.size();
        const T* vtx = NULL;
        double x0 = 0, y0 = 0, z0 = 0;
        double t;
        for (size_t i = 1; i < n; ++i) {
            vtx = points[i].buffer();
            t = 1.0 / double(i + 1);
            x0 = x0 * t * double(i) + vtx[0] * t;
            y0 = y0 * t * double(i) + vtx[1] * t;
            z0 = z0 * t * double(i) + vtx[2] * t;
        }

        double dx, dy, dz, w;
        double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        for (size_t i = 0; i < n; ++i) {
            vtx = points[i].buffer();
            dx = vtx[0] - x0;
            dy = vtx[1] - y0;
            dz = vtx[2] - z0;
            w = weights[i];
            xx += dx * dx * w;
            xy += dx * dy * w;
            xz += dx * dz * w;
            yy += dy * dy * w;
            yz += dy * dz * w;
            zz += dz * dz * w;
        }
        double data[] = {
            xx, xy, xz, 
            xy, yy, yz, 
            xz, yz, zz
        };

        Matrix<double> cov(3, 3, data);
        Matrix<double> cov1 = cov.clone();
        int rk = cov1.rank();
        if (rk == 3) {
            double e;
            Matrix<double> ev;
            cov.eigenMin(e, ev);
            Vector3d ev1(ev.buffer());
            plane._normal = Vector3<T>(ev1[0], ev1[1], ev1[2]);
            plane._c = ev1.dot(Vector3d(x0, y0, z0));
        } else {
            if (rk == 2) {
                if (std::abs(cov1[1][1] - 1.0) < 0.001) {
                    plane._normal[0] = cov1[0][2];
                    plane._normal[1] = cov1[1][2];
                    plane._normal[2] = -1;
                    plane._normal.normalize();
                    plane._c = plane._normal.dot(Vector3<T>(x0, y0, z0));
                } else if (std::abs(cov1[0][0] - 1.0) < 0.001) {
                    plane._normal[0] = cov1[0][1];
                    plane._normal[1] = -1;
                    plane._normal[2] = 0;
                    plane._normal.normalize();
                    plane._c = plane._normal.dot(Vector3<T>(x0, y0, z0));
                } else {
                    plane._normal[0] = 1;
                    plane._normal[1] = 0;
                    plane._normal[2] = 0;
                    plane._normal.normalize();
                    plane._c = plane._normal.dot(Vector3<T>(x0, y0, z0));
                }
            } else {
                //The plane can't be defined.
                return false;
            }
        }
        return true;
    }

    template <typename T>
    bool plane_fit_ransac(const std::vector<Point3<T> >& points,
        Plane<T>& plane_bestfit,
        std::vector<int>& inliers,
        double dist_error, int max_iter_time, double enough_good)
    {
        inliers.clear();
        bool found = false;
        int n = points.size();
        int iter_time = 0;

        const int n_enough_good = n * enough_good;
        
        std::srand(n);
        Point3<T> model[3];    
        SmartArray<bool> tag(n);
        int j;
        double err;
        while (iter_time < max_iter_time) {
            tag.reset(false);
            for (int k = 0; k < 3; ++k) {
                j = std::rand() % n;
                while (tag[j]) {
                    j = std::rand() % n;
                }
                model[k] = points[j];
                tag[j] = true;
            }
            Plane<T> plane;
            if (!plane.create(model[0], model[1], model[2])) {
                continue;
            }

            std::vector<int> inliers_temp;
            for (j = 0; j < n; ++j) {
                err = std::abs(plane.pointDistance(points[j]));
                if (err <= dist_error) {
                    inliers_temp.push_back(j);
                }
            }

            if (inliers_temp.size() > inliers.size()) {
                inliers = inliers_temp;
                plane_bestfit = plane;
				found = true;
				if (inliers_temp.size() >= n_enough_good) {
					break;
				}   
            }
            ++iter_time;
        }

//         if (found) {
//             std::vector<Point3<T> >inlier_samples;
//             for (int i = 0; i < inliers.size(); ++i) {
//                 inlier_samples.push_back(points[inliers[i]]);
//             }
//             return plane_fit_leastsquare(iner_samples, plane_bestfit);
//         }

        return found;
    }

    template<typename T>
    bool plane_fit_best(const std::vector<Point3<T> >& points, Plane<T>& plane, float inner_dist)
    {
        Plane<T> plane_best;
        int n_point = points.size();
        if (!plane_fit_leastsquare(points, plane_best)) {
            return false;
        }

        double thre = std::cos(Deg2Rad(0.001));
        double dt = 0;
        int time = 0;
        double dist_thre = 0.25;
        double dist = 0;
        while (time < 100 && dt < thre) {
            std::vector<Point3<T> > inners;
            std::vector<double> w;
            for (int i = 0; i < n_point; ++i) {
                dist = plane_best.pointDistance(points[i]);
                if (dist < dist_thre) {
                    w.push_back(std::exp(-dist));
                    inners.push_back(points[i]);
                }
            }

            plane_fit_leastsquare_w(inners, w, plane);
            dt = plane._normal.dot(plane_best._normal);
            plane_best = plane;
            ++time;
        }
        plane = plane_best;
        return true;
    }

    template MPCDPS_CORE_ITEM class Plane<float>;
    template MPCDPS_CORE_ITEM class Plane<double>;

    template MPCDPS_CORE_ITEM
    bool plane_fit_leastsquare(
        const std::vector<Point3<float> >& points, Plane<float>& plane);

    template MPCDPS_CORE_ITEM
    bool plane_fit_leastsquare(
        const std::vector<Point3<double> >& points, Plane<double>& plane);

    template MPCDPS_CORE_ITEM
        bool plane_fit_leastsquare_w(
            const std::vector<Point3<float> >& points, const std::vector<double>& weights, Plane<float>& plane);

    template MPCDPS_CORE_ITEM
        bool plane_fit_leastsquare_w(
            const std::vector<Point3<double> >& points, const std::vector<double>& weights, Plane<double>& plane);

    template MPCDPS_CORE_ITEM
        bool plane_fit_ransac(const std::vector<Point3<float> >& points,
        Plane<float>& plane_bestfit,
            std::vector<int>& inliers,
        double dist_error_threshold, int max_iter_time, double enough_good);

    template MPCDPS_CORE_ITEM
        bool plane_fit_ransac(const std::vector<Point3<double> >& points,
        Plane<double>& plane_bestfit,
            std::vector<int>& inliers,
        double dist_error_threshold, int max_iter_time, double enough_good);

    template MPCDPS_CORE_ITEM
        bool plane_fit_best(const std::vector<Point3<float> >& points,
            Plane<float>& plane, float inner_dist);

    template MPCDPS_CORE_ITEM
        bool plane_fit_best(const std::vector<Point3<double> >& points,
            Plane<double>& plane, float inner_dist);

}