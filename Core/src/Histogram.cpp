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

#include "Histogram.h"
#include <fstream>
#include <cmath>
#include <iostream>

namespace mpcdps {

    Histogram::Histogram()
    {

    }

    Histogram::~Histogram()
    {

    }

    void Histogram::initialize(double vmin, double vmax, int n_piece)
    {
        _xmin_pre = vmin - 1e-6;
        _xmax_pre = vmax + 1e-6;
        _n_pieces = n_piece;
        _dx = (_xmax_pre - _xmin_pre) / double(n_piece);
        _count = 0;
        _x_mean_bin.resize(n_piece, 0);
        _probilities_bin.resize(n_piece, 0);
    }

    void Histogram::add(double v)
    {
        int i = (v - _xmin_pre) / _dx;
        if (i < 0) i = 0;
        if (i >= _n_pieces)  i = _n_pieces - 1;
        double n_i = _probilities_bin[i];
        _x_mean_bin[i] = (_x_mean_bin[i] * n_i + v) / double(n_i + 1);
        _probilities_bin[i] = n_i + 1;
        if (v < _xmin)  _xmin = v;
        if (v > _xmax) _xmax = v;

        ++_count;
    }

    void Histogram::finish()
    {
        double t = 1.0 / double(_count);
        for (int i = 0; i < _probilities_bin.size(); ++i) {
            _probilities_bin[i] *= t;
        }
    }

    //-------------------------------------------------------------------------------------//
    int Histogram::get_bin_size() const
    {
        return _probilities_bin.size();
    }

    /*
    For bin i, x range is (x1, x2], this function return x2.
    */
    double Histogram::get_i_x_break(int i) const
    {
        return _xmin_pre + (i + 1) * _dx;
    }

    /*
    Return x_break while integral_probability(x_min, x_break) >= accum_probability
    */
    double Histogram::get_x_break(double sum_probability) const
    {
        int i = 0;
        double sum_p = _probilities_bin[i];
        while (sum_p < sum_probability && i < _probilities_bin.size()) {
            ++i;
            sum_p += _probilities_bin[i];
        }
        return get_i_x_break(i);
    }

    /*
    Return bin index for x.
    */
    int Histogram::get_i_bin(double x) const
    {
        int i = (x - _xmin_pre) / _dx;
        if (i < 0) {
            i = 0;
            std::cout << "mpcdps::Histogram warning: x less than x_min" << std::endl;
        }
        if (i >= _n_pieces) {
            i = _n_pieces - 1;
            std::cout << "mpcdps::Histogram warning: x larger than x_max" << std::endl;
        }
        return i;
    }

    /*
    Return probability for i'th bin.
    */
    double Histogram::get_i_probability(int i) const
    {
        return _probilities_bin[i];
    }

    /*
    Return mean x for i'th bin.
    */
    double Histogram::get_i_x_mean(int i) const
    {
        return _x_mean_bin[i];
    }

    double Histogram::get_mean() const
    {
        double m = 0;
        for (int i = 0; i < _x_mean_bin.size(); ++i) {
            m += _x_mean_bin[i] * _probilities_bin[i];
        }
        return m;
    }

    //var = expect((X - MEAN)^2)
    double Histogram::get_variance(double mean) const
    {
        double var = 0;
        for (int i = 0; i < _probilities_bin.size(); ++i) {
            double dx = _x_mean_bin[i] - mean;
            var += dx * dx * _probilities_bin[i];
        }
        return var;
    }

    std::ostream& operator << (std::ostream& out, const Histogram& histg)
    {
        int n = histg.get_bin_size();

        double mean = histg.get_mean();
        double sdv = std::sqrt(histg.get_variance(mean));

        out << "min: " << histg.get_min() << std::endl;
        out << "max: " << histg.get_max() << std::endl;
        out << "mean: " << mean << std::endl;
        out << "std: " << sdv << "\n" << std::endl;

        out << "i, x, mean_x, probability" << std::endl;
        double x = histg._xmin_pre;
        double dx = histg._dx;
        for (int i = 0; i < n; ++i, x += dx) {
            out << i << ", " << x << ", " << histg.get_i_x_mean(i) << ", " << histg.get_i_probability(i) << std::endl;
        }

        return out;
    }
}