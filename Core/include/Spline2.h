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

#ifndef   MPCDPS_SPLINE_H
#define  MPCDPS_SPLINE_H

#include "Matrix.h"
#include <vector>
#include <Point2.h>

namespace mpcdps {

	class MPCDPS_CORE_ITEM Spline
	{
	public:
		Spline();
		~Spline();

		enum BoundaryCondition
		{
			First_Deriv,
			Second_Deriv
		};

	//Note: x[i] < x[i+1]
		bool build(const std::vector<double>& XList, const std::vector<double>& YList,
			BoundaryCondition left_boundarycondition = Second_Deriv, double left_value = 0,
			BoundaryCondition right_boundarycondition = Second_Deriv, double right_value = 0);

		double getY(double x) const;
		double getFirstDerive(double x) const;
		double getSecondDerive(double x) const;

        //for knot i
		double getCurvature(int i) const;
		double getCurvature(double x) const;

        int getKnotCount() const;

		void clear();

	protected:
		int getIndex(double x) const;

	private:
		Matrix<double> mParams;
		std::vector<double> mXList;
	};

	class SplineFitCore;

	class MPCDPS_CORE_ITEM SplineFit
	{
	public:
		SplineFit();
		~SplineFit();

		bool build(const std::vector<Point2d>& ptList, double control_dist = 0.5);
		std::vector<Point2d> getSamplePoints(double sample_interval) const;
		Point2d getFitPoint(const Point2d& pt) const;

		double getCurvature(const Point2d& pt) const;

        const Spline* getSpline() const;

		void clear();

	protected:
		SplineFitCore* mCore;
	};

}

#endif