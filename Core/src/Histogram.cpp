#include "Histogram.h"
#include <fstream>
#include <cmath>
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

namespace mpcdps {

    Histogram::Histogram()
    {

    }

    Histogram::~Histogram()
    {

    }

    void Histogram::initialize(double vmin, double vmax, int n_piece)
    {
        _xmin = vmin - 1e-6;
        _xmax = vmax + 1e-6;
        _dx = (_xmax - _xmin) / double(n_piece);
        _probilities_bin.clear();
        _probilities_bin.resize(n_piece, 0);
        _x_mean_bin.clear();
        _x_mean_bin.resize(n_piece, 0);
        _x_mean = 0;
        _count = 0;
    }

    void Histogram::add(double v)
    {
        int i = (v - _xmin) / _dx;
        double n_i = _probilities_bin[i];
        _x_mean_bin[i] = (_x_mean_bin[i] * n_i + v) / double(n_i + 1);
        _probilities_bin[i] = n_i + 1;
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
        return _xmin + (i + 1) * _dx;
    }

    /*
    Return x_break while integral_probability(x_min, x_break) >= accum_probability
    */
    double Histogram::get_x_break(double sum_probability) const
    {
        double sum_p = 0;
        int i = 0;
        for (i = 0; i < _probilities_bin.size() && sum_p < sum_probability; ++i) {
            sum_p += _probilities_bin[i];
        }
        return get_i_x_break(i);
    }

    /*
    Return bin index for x.
    */
    int Histogram::get_i_bin(double x) const
    {
        return (x - _xmin) / _dx;
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
    double Histogram::get_i_x_mean(int i)
    {
        return _x_mean_bin[i];
    }

    /*
    Get global mean x.
    */
    double Histogram::get_x_mean()
    {
        if (std::abs(_x_mean) < 1e-8) {
            for (int i = 0; i < _probilities_bin.size(); ++i) {
                _x_mean += _probilities_bin[i] * _x_mean_bin[i];
            }
        }
        return _x_mean;
    }

    /*
    Get global std.
    */
    double Histogram::get_x_std()
    {
        if (std::abs(_x_mean) < 1e-8) {
            get_x_mean();
        }
        double dx = 0;
        double std = 0;
        for (int i = 0; i < _probilities_bin.size(); ++i) {
            dx = _x_mean_bin[i] - _x_mean;
            std += dx * dx * _probilities_bin[i];
        }
        return std;
    }

    /*
    Return p,  p = sum(p1, ..., pk) while k_x <= x
    */
    double Histogram::get_sum_probability(double x) const
    {
        double p_sum = 0;
        int x1 = _xmin;
        for (int i = 0; i < _probilities_bin.size() && x1 < x; ++i) {
            x1 += _dx;
            p_sum += _probilities_bin[i];
        }
        return p_sum;
    }

    void Histogram::output(const std::string& filepath, int presision) const
    {
        std::ofstream ofile(filepath);
        int n = _probilities_bin.size();
        ofile << "i, mean_x, x, probability" << std::endl;
        ofile.setf(std::ios::fixed | std::ios::left);
        ofile.precision(presision);

        double x = _xmin;
        for (int i = 0; i < n; ++i, x += _dx) {
            ofile << i << ", " << _x_mean_bin[i] << ", " << x << ", " << _probilities_bin[i] << std::endl;
        }
        ofile.close();
    }

}