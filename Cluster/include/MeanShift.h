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

/*
Reference papar:
http://blog.csdn.net/google19890102/article/details/37656733
http://www.cnblogs.com/chaosimple/p/3164775.html
*/

#ifndef  MPCDPS_MEAN_SHIFT_H
#define MPCDPS_MEAN_SHIFT_H

#include <cmath>
#include <KDTree.h>
#include <SmartPointer.h>
#include <PublicFunc.h>

namespace mpcdps {

    class KernelFunc
    {
    public:
        KernelFunc(double C = 2.0) :_C(C)
        {
        }

        /*
        g(r) = -k(r);
        r = ||(x-xi)/h||^2
        */
        virtual double g(double r) const
        {
            return _C * std::pow(M_E, -0.5*r) * std::sqrt(r);
        }

    protected:
        double _C;
    };

    template<typename T, int K>
    class MeanShiftCore
    {
    public:
        MeanShiftCore()
        {
            for (int i = 0; i < K; ++i) {
                _weight[i] = 1;
            }
            _kernel = new KernelFunc;
        }

        ~MeanShiftCore()
        {
        }

        void initialize(const SmartArray2D<T, K>& vtx_array, T vmin[K], T vmax[K], 
            std::vector<int> target_points = std::vector<int>())
        {
            if (target_points.empty()) {
                target_points = make_vector<int>(vtx_array.size());
            }

            double nodesize = 2.0;
            T ns[K];
            for (int i = 0; i < K; ++i) {
                ns[i] = nodesize;
            }

            _kdtree.setElements(vtx_array);
            _kdtree.build(target_points, vmin, vmax, ns);
        }

        void setKernel(KernelFunc* kernel)
        {
            _kernel = kernel;
        }

        //default: 1 for each dimension.
        void setWeightVector(const T w[])
        {
            for (int i = 0; i < K; ++i) {
                _weight[i] = w[i];
            }
        }

        /*Set search radius. */
        void setSearchRadius(float radius)
        {
            _search_radius = radius;
        }

        //default: 0.01
        void setConvergenceDistance(double dist)
        {
            _convergence_dist2 = dist * dist;
        }

        void meanShift(const T* point, T* model);

    protected:
        double distance2(T* point, T* point1) const
        {
            double dist = 0;
            for (int i = 0; i < K; ++i) {
                dist += (point[i] - point1[i]) * (point[i] - point1[i]) * _weight[i];
            }
            return dist;
        }

    protected:
        double _search_radius;
        SmartPointer<KernelFunc> _kernel;
        T _weight[K];
        KDTree<T, K> _kdtree;

        double _convergence_dist2 = 0.0001;
    };

    template<typename T, int K>
    void MeanShiftCore<T, K>::meanShift(const T* point, T* mode)
    {
        T mean[K];

        for (int i = 0; i < K; ++i) {
            mode[i] = point[i];
        }

        std::vector<int> neighbs;
        std::vector<double> dist2s;

        const double h2 = _search_radius * _search_radius;
        double dist2 = 0;
        double z = 0;
        double w = 0;
        double sw = 0;
        const T* neigb = NULL;

        double dist2_pre = DBL_MAX;

        int k = 0;
        while (1) {
            neighbs = _kdtree.searchRadius(mode, _search_radius, dist2s);
            if (neighbs.empty())
                break;

            for (int i = 0; i < K; ++i) {
                mean[i] = 0;
            }

            sw = 0;
            for (int i = 0; i < neighbs.size(); ++i) {
                neigb = _kdtree.getElement(neighbs[i]);

                //dist2 = distance2(mode, neigb);
                dist2 = dist2s[i];
                z = dist2 / h2;
                w = _kernel->g(z);
                for (int j = 0; j < K; ++j) {
                    mean[j] += neigb[j] * w;
                }
                sw += w;
            }

            for (int j = 0; j < K; ++j) {
                mean[j] /= sw;
            }

            dist2 = distance2(mode, mean);
            if (dist2 >= dist2_pre)
                break;

            if (dist2 <= _convergence_dist2)
                break;

            dist2_pre = dist2;
            ++k;
        }

        for (int i = 0; i < K; ++i) {
            mode[i] = mean[i];
        }
    }
}

#endif