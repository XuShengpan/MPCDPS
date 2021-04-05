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

#include "Histogram.h"
#include <fstream>
#include <cmath>
#include <iostream>
#include <PublicInfo.h>

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
        _sample_count_bin.resize(n_piece, 0);
    }

    void Histogram::add(double v)
    {
        int i = (v - _xmin_pre) / _dx;
        if (i < 0) i = 0;
        if (i >= _n_pieces)  i = _n_pieces - 1;
        uint n_i = _sample_count_bin[i];
        _x_mean_bin[i] = (n_i * _x_mean_bin[i] + v) / double(n_i + 1);
        _sample_count_bin[i] = n_i + 1;
        if (v < _xmin)  _xmin = v;
        if (v > _xmax) _xmax = v;

        ++_count;
    }

    void Histogram::finish()
    {
    }

    //-------------------------------------------------------------------------------------//
    int Histogram::get_bin_size() const
    {
        return _sample_count_bin.size();
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
        uint n_sum = _sample_count_bin[i];
		uint n_max = _count * sum_probability;
        while (n_sum < n_max && i < _sample_count_bin.size()) {
            ++i;
            n_sum += _sample_count_bin[i];
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
		return _sample_count_bin[i] / double(_count);
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
		double ratio = 1.0 / double(_count);
        for (int i = 0; i < _x_mean_bin.size(); ++i) {
			m += _x_mean_bin[i] * _sample_count_bin[i] * ratio;
        }
        return m;
    }

	double Histogram::get_variance(double mean) const
	{
		double var = 0;
		double ratio = 1.0 / double(_count);
		double dx = 0;
		for (int i = 0; i < _sample_count_bin.size(); ++i) {
			dx = _x_mean_bin[i] - mean;
			var += dx * dx * _sample_count_bin[i] * ratio;
		}
		return var;
	}

	double Histogram::get_variance() const
	{
		double mean = get_mean();
		return get_variance(mean);
	}

    std::ostream& operator << (std::ostream& out, const Histogram& histg)
    {
        int n = histg.get_bin_size();

        double mean = histg.get_mean();
        double sdv = std::sqrt(histg.get_variance(mean));

        out << "min," << histg.get_min() << std::endl;
        out << "max," << histg.get_max() << std::endl;
        out << "mean," << mean << std::endl;
        out << "std," << sdv << "\n" << std::endl;

        out << "i,x,mean_x,frequency" << std::endl;
        double x = histg._xmin_pre;
        double dx = histg._dx;
        for (int i = 0; i < n; ++i, x += dx) {
            out << i << "," << x << "," << histg.get_i_x_mean(i) << "," << histg._sample_count_bin[i] << std::endl;
        }

        return out;
    }
}