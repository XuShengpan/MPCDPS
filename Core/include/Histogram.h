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

#ifndef  MPCDPS_HISTOGRAM_H
#define MPCDPS_HISTOGRAM_H

#include <vector>
#include <string>
#include "MPCDPSCoreLib.h"

namespace mpcdps {

	class MPCDPS_CORE_ITEM Histogram
	{
	public:
		Histogram();
		~Histogram();

		void initialize(double vmin, double vmax, int n_piece);
		void add(double v);
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

        /*
           Return mean x for i'th bin.
        */
        double get_i_x_mean(int i);

        /*
           Get global mean x.
        */
		double get_x_mean();

        /*
           Get global std.
        */
        double get_x_std();
        
        /*  
           Return p,  p = sum(p1, ..., pk) while x_break_k <= x
        */
        double get_sum_probability(double x) const;

        void output(const std::string& filepath, int presision = 4) const;

	protected:
		double _xmin;
		double _xmax;
		double _dx;
        double _x_mean = 0;
		int _count;
        std::vector<double> _x_mean_bin;
		std::vector<double> _probilities_bin;
	};
}

#endif