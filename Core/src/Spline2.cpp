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

#include "Spline2.h"
#include <algorithm>
#include "PublicFunc.h"

namespace mpcdps {

	Spline::Spline()
	{

	}

	Spline::~Spline()
	{

	}

	int Spline::getKnotCount() const
	{
		return mXList.size();
	}

	//XList should be sorted first.
	bool Spline::build(const std::vector<double>& XList, const std::vector<double>& YList,
		BoundaryCondition left_boundarycondition, double left_value,
		BoundaryCondition right_boundarycondition, double right_value)
	{

		const size_t n = XList.size();
		mXList = XList;

		uint i;
		std::vector<double> dx(n), dy(n);
		for(i=0; i< n-1 ; ++i) {
			dx[i] = XList[i+1] - XList[i];
			dy[i] = YList[i+1] - YList[i];
			assert(dx[i] > 0);
		}

		Matrix<double> A(n, n), b(n, 1), A1;
		A.reset(0);
		b.reset(0);

		for(i = 1;i< n-1; ++i) {
			A[i][i-1] = dx[i-1]/3.0;
			A[i][i] = (dx[i-1]+dx[i])*2.0/3.0;
			A[i][i+1] = dx[i]/3.0;
			b[i][0] = dy[i]/dx[i] - dy[i-1]/dx[i-1];
		}

		if(left_boundarycondition == Second_Deriv) {
			A[0][0] = 2.0;
			b[0][0] = left_value;
		} else {
			A[0][0] = 2.0*dx[0];
			A[0][1] = 1.0*dx[0];
			b[0][0] = 3.0*(dy[0]/dx[0] - left_value);
		}

		if(right_boundarycondition == Second_Deriv) {
			A[n-1][n-1] = 2.0;
			b[n-1][0] = right_value;
		} else {
			A[n-1][n-1] = 2.0*dx[n-1];
			A[n-1][n-2] = 1.0*dx[n-1];
			b[n-1][0] = 3.0*(right_value - dy[n-1]/dx[n-1]);
		}

		bool tag;
		A1 = A.inverseMatrix(&tag);
		if(!tag)
			return false;
		b = A1*b;

		mParams.resize(n, 4);
		for(i = 0; i < n-1; ++i) {
			mParams[i][0] = (b[i+1][0] - b[i][0])/dx[i]/3.0;
			mParams[i][1] = b[i][0];
			mParams[i][2] = dy[i]/dx[i] - (2.0*b[i][0] + b[i+1][0])*dx[i]/3.0;
			mParams[i][3] = YList[i];
		}

		mParams[n-1][0] = 0;
		mParams[n-1][1] = right_value;
		mParams[n-1][2] = 3.0*mParams[n-2][0]*dx[n-2]*dx[n-2]+2.0*mParams[n-2][1]*dx[n-2]+mParams[n-2][2];   // = f'_{n-2}(x_{n-1})
		mParams[n-1][3] = YList[n-1];

		return true;
	}

	int Spline::getIndex(double x) const
	{
		// find the closest point m_x[idx] < x, idx=0 even if x<m_x[0]
		std::vector<double>::const_iterator it;
		it=std::lower_bound(mXList.begin(),mXList.end(),x);
		return std::max(int(it-mXList.begin())-1, 0);
	}

	double Spline::getY(double x) const
	{
		size_t n=mXList.size();
		int idx = getIndex(x);
		if(idx<0) idx = 0;

		double h=x-mXList[idx];
		double interpol;
		if(x<mXList[0]) {
			// extrapolation to the left
			interpol=(mParams[0][1]*h + mParams[0][2])*h + mParams[0][3];
		} else if(x>mXList[n-1]) {
			// extrapolation to the right
			interpol=(mParams[n-1][1]*h + mParams[n-1][2])*h + mParams[n-1][3];
		} else {
			// interpolation
			interpol=((mParams[idx][0]*h + mParams[idx][1])*h + mParams[idx][2])*h + mParams[idx][3];
		}
		return interpol;
	}

	double Spline::getFirstDerive(double x) const
	{
		size_t n=mXList.size();
		int idx = getIndex(x);
		if(idx<0) idx = 0;

		double h=x-mXList[idx];
		double t;
		if(x<mXList[0]) {
			t = 2.0*mParams[0][1]*h + mParams[0][2];
		} else if(x>mXList[n-1]) {
			// extrapolation to the right
			t = 2.0*mParams[n-1][1]*h + mParams[n-1][2];
		} else {
			// interpolation
			t = 3.0*mParams[idx][0]*h*h + 2.0*mParams[idx][1]*h + mParams[idx][2];
		}
		return t;
	}

	double Spline::getSecondDerive(double x) const
	{
		size_t n=mXList.size();
		int idx = getIndex(x);
		if(idx<0) idx = 0;

		double h=x-mXList[idx];
		double t;
		if(x<mXList[0]) {
			t = 2.0*mParams[0][1];
		} else if(x>mXList[n-1]) {
			// extrapolation to the right
			t = 2.0*mParams[n-1][1];
		} else {
			// interpolation
			t = 6.0*mParams[idx][0]*h + 2.0*mParams[idx][1];
		}
		return t;
	}

	void Spline::clear()
	{
		mParams.clear();
		mXList.clear();
	}

	double Spline::getCurvature(int i) const
	{
		return getCurvature(mXList[i]);
	}

	double Spline::getCurvature(double x) const
	{
		double f1 = getFirstDerive(x);
		double f2 = getSecondDerive(x);
		return std::abs(f2)*std::pow(1.0+Square(f1),-3.0/2.0);
	}

	//-------------------------------------------------------------------------------------//

	class SplineFitCore
	{
	public:
		SplineFitCore() {}

		Spline mSpline;
		double mMin;
		double mMax;
		int mType;  //0:x, 1:y
	};

	SplineFit::SplineFit()
	{
		mCore = new SplineFitCore;
	}

	SplineFit::~SplineFit()
	{
		delete mCore;
	}

	bool SplineFit::build(const std::vector<Point2d>& ptList, double control_dist)
	{
		mCore->mSpline.clear();
		const size_t n = ptList.size();
		std::vector<int> ptIds(n);
		std::vector<double> xes(n), ys(n);
		uint i = 0;
		double xmin = DBL_MAX, xmax = -DBL_MAX;
		double ymin = DBL_MAX, ymax = -DBL_MAX;

		for(i = 0; i< n; ++i) {
			ptIds[i] = i;
			xes[i] = ptList[i][0];
			ys[i] = ptList[i][1];

			xmin = MINV(xes[i], xmin);
			xmax = MAXV(xes[i], xmax);

			ymin = MINV(ys[i], ymin);
			ymax = MAXV(ys[i], ymax);
		}

		std::vector<double> xList, yList;

		if(xmax - xmin >= ymax - ymin) {
			sort_shell_syn(xes, ptIds);
			mCore->mMin = xmin;
			mCore->mMax = xmax;
			mCore->mType = 1;

			double x0 = -DBL_MAX;
			Point2d pt;
			for(i=0; i< n; ++i) {
				if(xes[i] - x0 > control_dist) {
					x0 = xes[i];
					pt = ptList[ptIds[i]];
					xList.push_back(pt[0]);
					yList.push_back(pt[1]);
				}
			}

			if(xList.size()<3)
				return false;
			return mCore->mSpline.build(xList, yList);
		} else {
			mCore->mMin = ymin;
			mCore->mMax = ymax;
			mCore->mType = 2;
			sort_shell_syn(ys, ptIds);
			double y0 = -DBL_MAX;
			Point2d pt;
			for(i=0; i< n; ++i) {
				if(ys[i] - y0 > control_dist) {
					y0 = ys[i];
					pt = ptList[ptIds[i]];
					xList.push_back(pt[0]);
					yList.push_back(pt[1]);
				}
			}

			if(xList.size()<3)
				return false;

			return mCore->mSpline.build(yList, xList);
		}
	}

	std::vector<Point2d> SplineFit::getSamplePoints(double sample_interval) const
	{
		double x = mCore->mMin;
		double k, dx, y;
		std::vector<Point2d> ptList;
		while(x<=mCore->mMax) {
			y = mCore->mSpline.getY(x);
			if(mCore->mType == 1)
				ptList.push_back(Point2d(x,y));
			else
				ptList.push_back(Point2d(y,x));

			k = mCore->mSpline.getFirstDerive(x);
			dx = std::sqrt(sample_interval*sample_interval/(1.0 + k*k));
			x += dx;
		}
		return ptList;
	}

	Point2d SplineFit::getFitPoint(const Point2d& pt) const
	{
		assert(mCore->mType);
		if(mCore->mType == 1)
			return Point2d(pt[0], mCore->mSpline.getY(pt[0]));
		else  //2
			return Point2d(mCore->mSpline.getY(pt[1]), pt[1]);
	}

	void SplineFit::clear()
	{
		mCore->mSpline.clear();
		mCore->mType = 0;
	}

	const Spline* SplineFit::getSpline() const
	{
		return &mCore->mSpline;
	}

	double SplineFit::getCurvature(const Point2d& pt) const
	{
		if(mCore->mType == 1)
			return mCore->mSpline.getCurvature(pt[0]);
		else
			return mCore->mSpline.getCurvature(pt[1]);
	}
}