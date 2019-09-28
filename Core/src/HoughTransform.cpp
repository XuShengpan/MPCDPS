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

#include "HoughTransform.h"
#include <SmartArrayReal2D.h>
#include <SmartArray2D.h>

namespace mpcdps {

    template <typename T>
    int hough_transform_detect_line_core(
        const std::vector<Point2<T> >& points,
        Line2<T>& line,
        float angle_granularity_deg,
        float dist_granularity)
    {
        float angle_min = 0;
        float angle_max = 180;
        double d_max = 0;
        int n_point = points.size();
        for (int i = 0; i < n_point; ++i) {
            const auto p = points[i];
            double d2 = p[0] * p[0] + p[1] * p[1];
            if (d2 > d_max) {
                d_max = d2;
            }
        }
        d_max = std::sqrt(d_max);
        float d_min = -d_max;

        int n_d = (d_max - d_min) / dist_granularity + 1;
        int n_theta = (angle_max - angle_min) / angle_granularity_deg + 1;
        SmartArrayReal2D<int> mat(n_theta, n_d);
        mat.reset(0);
        Vector2<int> index;
        int n_max = 0;
        for (int i = 0; i < n_theta; ++i) {
            float theta = angle_min + (i + 0.5)*angle_granularity_deg;
            float rad = Deg2Rad(theta);
            float sin_theta = std::sin(rad);
            float cos_theta = std::cos(rad);
            for (int k = 0; k < points.size(); ++k) {
                float d = points[k][0] * cos_theta + points[k][1] * sin_theta;
                int j = (d - d_min) / dist_granularity + 0.5;
                mat[i][j]++;

                if (mat[i][j] > n_max) {
                    n_max = mat[i][j];
                    index[0] = i;
                    index[1] = j;
                }
            }
        }

        int i = index[0];
        int j = index[1];
        float angle = angle_min + (i + 0.5) * angle_granularity_deg;
        float rad = Deg2Rad(angle);
        float d = d_min + (j + 0.5)*dist_granularity;
        line._a = std::cos(rad);
        line._b = std::sin(rad);
        line._c = d;
        return n_max;
    }

    template <typename T>
    void hough_transform_detect_line(
        const std::vector<Point2<T> >& points,
        std::vector<Line2<T> >& lines,
        float angle_granularity_deg,
        float dist_granularity,
        int min_point)
    {
        std::vector<Point2<T> > points_remain = points;

        while (1) {
            Line2<T> line;
            int n_inlier = hough_transform_detect_line_core(points, line,
                angle_granularity_deg, dist_granularity);

            if (n_inlier >= min_point) {
                lines.push_back(line);
                std::vector<Point2<T> > points1;
                for (int i = 0; i < points_remain.size(); ++i) {
                    auto pt = points_remain[i];
                    float dist = line.pointDistance(pt);
                    if (std::abs(dist) >= dist_granularity) {
                        points1.push_back(pt);
                    }
                }
                points_remain = points1;
            } else {
                break;
            }
        }
    }

    template <typename T>
    void hough_transform_detect_line(
        const std::vector<Point2<T> >& points,
        std::vector<Line2<T> >& lines,
        std::vector<std::vector<int> >& line_points,
        float angle_granularity_deg,
        float dist_granularity,
        int min_point,
        float min_length)
    {
        double d_max = 0;
        int n_point = points.size();
        for (int i = 0; i < n_point; ++i) {
            const auto p = points[i];
            double d2 = p[0] * p[0] + p[1] * p[1];
            if (d2 > d_max) {
                d_max = d2;
            }
        }
        d_max = std::sqrt(d_max);
        float d_min = -d_max;
        float angle_min = 0;
        float angle_max = 180;
        
        int n_d = (d_max - d_min) / dist_granularity + 1;
        int n_theta = (angle_max - angle_min) / angle_granularity_deg + 1;
        SmartArrayReal2D<std::vector<int> > mat(n_theta, n_d);
        Vector2<int> index;
        for (int i = 0; i < n_theta; ++i) {
            float theta = angle_min + (i + 0.5)*angle_granularity_deg;
            float rad = Deg2Rad(theta);
            float sin_theta = std::sin(rad);
            float cos_theta = std::cos(rad);
            for (int k = 0; k < points.size(); ++k) {
                float d = points[k][0] * cos_theta + points[k][1] * sin_theta;
                int j = (d - d_min) / dist_granularity + 0.5;
                mat[i][j].push_back(k);
            }
        }

        SmartArrayReal2D<bool> valid_tag(n_theta, n_d);
        valid_tag.reset(false);
        for (int i = 0; i < n_theta; ++i) {
            float theta = angle_min + (i + 0.5)*angle_granularity_deg;
            float rad = Deg2Rad(theta);
            float sin_theta = std::sin(rad);
            float cos_theta = std::cos(rad);
            for (int j = 0; j < n_d; ++j) {
                if (mat[i][j].size() >= min_point) {
                    Vector2<T> dir(sin_theta, -cos_theta);
                    float e_min = FLT_MAX;
                    float e_max = -FLT_MAX;
                    for (auto it : mat[i][j]) {
                        Vector2<T> v = points[it] - Point2<T>(0, 0);
                        float e = v.dot(dir);
                        if (e < e_min)  e_min = e;
                        if (e > e_max)  e_max = e;
                    }
                    float d = e_max - e_min;
                    if (d >= min_length) {
                        valid_tag[i][j] = true;
                    }
                }
            }
        }

        std::vector<std::map<int, int> > point_label_count;
        point_label_count.resize(points.size());
        for (int i = 0; i < n_theta; ++i) {
            for (int j = 0; j < n_d; ++j) {
                if (valid_tag[i][j]) {
                    int k = i * n_d + j;
                    for (int ptid : mat[i][j]) {
                        point_label_count[ptid][k] = mat[i][j].size();
                    }
                }
            }
        }

        std::map<int, std::vector<int> > label_line_points;

        for (int i_pt = 0; i_pt < points.size(); ++i_pt) {
            if(point_label_count[i_pt].empty()) continue;
            int lable_best = -1;
            int n_point = 0;
            for (auto iter : point_label_count[i_pt]) {
                if (iter.second > n_point) {
                    n_point = iter.second;
                    lable_best = iter.first;
                }
            }
            label_line_points[lable_best].push_back(i_pt);
        }

        lines.clear();
        line_points.clear();

        for (auto iter : label_line_points) {
            if(iter.second.size() < min_point) continue;
            int k = iter.first;
            int i = k / n_d;
            int j = k % n_d;
            float theta = angle_min + (i + 0.5)*angle_granularity_deg;
            float rad = Deg2Rad(theta);
            float sin_theta = std::sin(rad);
            float cos_theta = std::cos(rad);
            double d = d_min + (j + 0.5) * dist_granularity;
            Line2<T> line;
            line._a = cos_theta;
            line._b = sin_theta;
            line._c = d;
            lines.push_back(line);
            line_points.push_back(iter.second);
        }
    }

    /**@hough_transform_detect_plane*/
    template <typename T>
    int hough_transform_detect_plane_core(
        const std::vector<Point3<T> >& points,
        Plane<T>& plane,
        float angle_granularity_deg,
        float dist_granularity)
    {
        double d_max = 0;
        int n_point = points.size();
        for (int i = 0; i < n_point; ++i) {
            const auto p = points[i];
            double d2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
            if (d2 > d_max) {
                d_max = d2;
            }
        }

        d_max = std::sqrt(d_max);
        float d_min = -d_max;

        float v_angle_min = 0;
        float v_angle_max = 90;
        float h_angle_min = 0;
        float h_angle_max = 360;

        int nh = (h_angle_max - h_angle_min) / angle_granularity_deg + 1;
        int nv = (v_angle_max - v_angle_min) / angle_granularity_deg + 1;
        int nd = (d_max - d_min) / dist_granularity + 1;

        SmartArrayReal2D<std::vector<int> > mat(nh, nv);
        for (int i = 0; i < nh; ++i) {
            for (int j = 0; j < nv; ++j) {
                mat[i][j].resize(nd, 0);
            }
        }

        Vector3<int> index;
        int n_max = 0;

        for (int i = 0; i < n_point; ++i) {
            const auto point = points.at(i);
            for (int ih = 0; ih < nh; ih++) {
                float h_angle = h_angle_min + (ih + 0.5) * angle_granularity_deg;
                float h_rad = Deg2Rad(h_angle);
                float sin_h_angle = std::sin(h_rad);
                float cos_h_angle = std::cos(h_rad);
                for (int iv = 0; iv < nv; ++iv) {
                    float v_angle = v_angle_min + (iv + 0.5) * angle_granularity_deg;
                    float v_rad = Deg2Rad(v_angle);
                    float sin_v_angle = std::sin(v_rad);
                    float cos_v_angle = std::cos(v_rad);

                    float d = point[0] * cos_v_angle * cos_h_angle + 
                                 point[1] * cos_v_angle * sin_h_angle + 
                                 point[2] * sin_v_angle;
                    int id = (d - d_min) / dist_granularity;
                    mat[ih][iv][id] ++;

                    if (mat[ih][iv][id] > n_max) {
                        n_max = mat[ih][iv][id];
                        index[0] = ih;
                        index[1] = iv;
                        index[2] = id;
                    }
                }
            }
        }

        int i = index[0];
        int j = index[1];
        int k = index[2];

        float h_angle = h_angle_min + (i + 0.5)* angle_granularity_deg;
        float h_rad = Deg2Rad(h_angle);
        float sin_h_angle = std::sin(h_rad);
        float cos_h_angle = std::cos(h_rad);

        float v_angle = v_angle_min + (j + 0.5)* angle_granularity_deg;
        float v_rad = Deg2Rad(v_angle);
        float sin_v_angle = std::sin(v_rad);
        float cos_v_angle = std::cos(v_rad);

        float d = d_min + (k + 0.5) * dist_granularity;

        plane._normal = Vector3<T>(cos_v_angle * cos_h_angle, cos_v_angle * sin_h_angle, sin_v_angle);
        plane._c = d;
        
        return n_max;
    }

    /**@hough_transform_detect_plane*/
    template <typename T>
    void hough_transform_detect_plane(
        const std::vector<Point3<T> >& points,
        std::vector<Plane<T> >& planes,
        float angle_granularity_deg,
        float dist_granularity,
        int min_point)
    {
        std::vector<Point3<T> > points_remain = points;

        while (1) {
            Plane<T> plane;
            int n_inlier = hough_transform_detect_plane_core(points_remain, plane,
                angle_granularity_deg, dist_granularity);

            if (n_inlier >= min_point) {
                planes.push_back(plane);
                std::vector<Point3<T> > points1;
                for (int i = 0; i < points_remain.size(); ++i) {
                    auto pt = points_remain[i];
                    float dist = plane.pointDistance(pt);
                    if (std::abs(dist) >= dist_granularity) {
                        points1.push_back(pt);
                    }
                }
                points_remain = points1;
            } else {
                break;
            }
        }
    }

    //---------------------------------------------------------------------------------------------------//

    template <typename T>
    int hough_transform_detect_circle_core(
        const std::vector<Point2<T> >& points,
        Circle<T>& circle,
        float r_min,
        float r_max,
        float dist_granularity)
    {
        std::vector<int> inliers;
        if (!circle_fit_ransac(points, circle, inliers)) {
            return 0;
        }
        if (circle._radius > r_max + r_min) {
            return 0;
        }

        float xmin = circle._center[0] - circle._radius;
        float xmax = circle._center[0] + circle._radius;
        float ymin = circle._center[1] - circle._radius;
        float ymax = circle._center[1] + circle._radius;

        int nx = circle._radius / dist_granularity + 1;
        int ny = nx;
        int nr = (r_max - r_min) / dist_granularity + 1;
        
        SmartArrayReal2D<std::vector<int> > matrix(nx, ny);  //ix, iy, ir
        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                matrix[ix][iy].resize(nr, 0);
            }
        }

        Vector3<int> index(-1, -1, -1);
        int n_max = 0;
        for (int ix = 0; ix < nx; ++ix) {
            float x0 = xmin + (ix + 0.5) * dist_granularity;
            for (int iy = 0; iy < ny; ++iy) {
                float y0 = ymin + (iy + 0.5) * dist_granularity;
                Point2<T> p(x0, y0);
                for (int i = 0; i < points.size(); ++i) {
                    Vector2<T> v = p - points[i];
                    float r = v.norm();
                    int ir = (r - r_min)/dist_granularity;
                    if(ir < 0 || ir >= nr) continue;

                    matrix[ix][iy][ir]++;
                    if (matrix[ix][iy][ir] > n_max) {
                        n_max = matrix[ix][iy][ir];
                        index[0] = ix;
                        index[1] = iy;
                        index[2] = ir;
                    }
                }
            }
        }

        float x = xmin + (index[0] + 0.5) * dist_granularity;
        float y = ymin + (index[1] + 0.5) * dist_granularity;
        float r = r_min + (index[2] + 0.5) * dist_granularity;
        circle._center = Point2<T>(x, y);
        circle._radius = r;

        return n_max;
    }

    //---------------------------------------------------------------------------------------------------//

    template MPCDPS_CORE_ITEM
        int hough_transform_detect_line_core(
            const std::vector<Point2<float> >& points,
            Line2<float>& line,
            float angle_granularity_deg,
            float dist_granularity);

    template MPCDPS_CORE_ITEM
        int hough_transform_detect_line_core(
            const std::vector<Point2<double> >& points,
            Line2<double>& line,
            float angle_granularity_deg,
            float dist_granularity);

    template MPCDPS_CORE_ITEM
        void hough_transform_detect_line(
            const std::vector<Point2<float> >& points,
            std::vector<Line2<float> >& lines,
            float angle_granularity_deg,
            float dist_granularity,
            int min_point);

    template MPCDPS_CORE_ITEM
        void hough_transform_detect_line(
            const std::vector<Point2<double> >& points,
            std::vector<Line2<double> >& lines,
            float angle_granularity_deg,
            float dist_granularity,
            int min_point);

    //---------------------------------------------------------------------------------------------------//

    template MPCDPS_CORE_ITEM
        void hough_transform_detect_line(
            const std::vector<Point2<float> >& points,
            std::vector<Line2<float> >& lines,
            std::vector<std::vector<int> >& line_points,
            float angle_granularity_deg,
            float dist_granularity,
            int min_point,
            float min_length);

    template MPCDPS_CORE_ITEM
        void hough_transform_detect_line(
            const std::vector<Point2<double> >& points,
            std::vector<Line2<double> >& lines,
            std::vector<std::vector<int> >& line_points,
            float angle_granularity_deg,
            float dist_granularity,
            int min_point,
            float min_length);

    //---------------------------------------------------------------------------------------------------//
    template MPCDPS_CORE_ITEM
    int hough_transform_detect_plane_core(
        const std::vector<Point3<float> >& points,
        Plane<float>& plane,
        float angle_granularity_deg,
        float dist_granularity);

    template MPCDPS_CORE_ITEM
    int hough_transform_detect_plane_core(
        const std::vector<Point3<double> >& points,
        Plane<double>& plane,
        float angle_granularity_deg,
        float dist_granularity);

    template MPCDPS_CORE_ITEM
        void hough_transform_detect_plane(
            const std::vector<Point3<float> >& points,
            std::vector<Plane<float> >& planes,
            float angle_granularity_deg,
            float dist_granularity,
            int min_point);

    template MPCDPS_CORE_ITEM
        void hough_transform_detect_plane(
            const std::vector<Point3<double> >& points,
            std::vector<Plane<double> >& planes,
            float angle_granularity_deg,
            float dist_granularity,
            int min_point);

    //---------------------------------------------------------------------------------------------------//

    template MPCDPS_CORE_ITEM
    int hough_transform_detect_circle_core(
        const std::vector<Point2<float> >& points,
        Circle<float>& circle,
        float r_min,
        float r_max,
        float dist_granularity);

    template MPCDPS_CORE_ITEM
        int hough_transform_detect_circle_core(
            const std::vector<Point2<double> >& points,
            Circle<double>& circle,
            float r_min,
            float r_max,
            float dist_granularity);
}