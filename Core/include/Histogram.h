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

#ifndef  MPCDPS_HISTOGRAM_H
#define MPCDPS_HISTOGRAM_H

#include <vector>
#include <ostream>
#include <cfloat>
#include "MPCDPSCoreLib.h"

namespace mpcdps {

	class MPCDPS_CORE_ITEM Histogram
	{
	public:
		Histogram();
		~Histogram();

		void initialize(double vmin, double vmax, int n_piece);

		void add(double v);

	    //Note: Obsoleted. There is no need to call this function.
        void finish();

		int get_bin_size() const;

        /*
        For bin i, x range is (x1, x2], this function return x2.
        */
        double get_i_x_break(int i) const;

        /*
           Return x_break while integral_probability(x_min, x_break) >= accum_probability
        */
		double get_x_break(double sum_probability) const;

        /* 
           Return bin index for x.
        */
        int       get_i_bin(double x) const;

        /*
           Return probability for i'th bin.
        */
        double get_i_probability(int i) const;

        unsigned int get_i_frequency(int i) const {return _sample_count_bin[i];}

        /*
           Return mean x for i'th bin.
        */
        double get_i_x_mean(int i) const;

        double get_mean() const;
		double get_variance() const;
        double get_min() const { return _xmin; }
        double get_max() const { return _xmax; }
        double get_dx() const { return _dx; }

        friend MPCDPS_CORE_ITEM std::ostream& operator << (std::ostream& out, const Histogram& histg);

	protected:
		double get_variance(double mean) const;

	protected:
		double _xmin_pre;
		double _xmax_pre;

        double _xmin = DBL_MAX;
        double _xmax = -DBL_MAX;

		double _dx;
		int _count;
        int _n_pieces;
        std::vector<double> _x_mean_bin;
		std::vector<unsigned int> _sample_count_bin;
	};
}

#endif
