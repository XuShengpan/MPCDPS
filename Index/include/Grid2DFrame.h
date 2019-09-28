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

#ifndef  MPCDPS_GRID2FRAME_H
#define MPCDPS_GRID2FRAME_H

#include <cmath>
#include "PublicInfo.h"

namespace mpcdps 
{

    /* 
        \note
        dimension 0:  r, related with y
        dimension 1:  c, related with x
    */
	class Grid2DFrame
	{
	public:
		Grid2DFrame():_dx(0), _dy(0), _rn(0), _cn(0)
		{

		}

		~Grid2DFrame()
		{

		}

		Grid2DFrame(const Grid2DFrame& obj):_x0(obj._x0), _y0(obj._y0), _dx(obj._dx), _dy(obj._dy), _rn(obj._rn), _cn(obj._cn)
		{

		}

		void initialize1(double xStart, double yStart, double xLen, double yLen, double dx, double dy)
		{
			_x0 = xStart;
			_y0 = yStart;
			_dx = dx;
			_dy = dy;

			_rn = yLen/std::abs(dy) + 1;
			_cn = xLen/std::abs(dx) + 1;
		}

		void initialize2(double xStart, double yStart, double dx, double dy, int rn, int cn)
		{
			_x0 = xStart;
			_y0 = yStart;
			_dx = dx;
			_dy = dy;
			_rn = rn;
			_cn = cn;
		}

		int get_r(double y)  const
		{
			return LOCATE(y,_y0, _dy);
		}

		int get_c(double x)  const
		{
			return LOCATE(x, _x0, _dx);
		}

		int rowCount() const
		{
			return _rn;
		}

		int colCount()  const
		{
			return _cn;
		}

		double get_xStart() const
		{
			return _x0;
		}

		double get_yStart() const
		{
			return _y0;
		}

		double get_x(int c) const
		{
			return SKIP(_x0, _dx, c);
		}

		double get_y(int r) const
		{
			return SKIP(_y0, _dy, r);
		}

		float get_dx() const { return _dx;}
		float get_dy() const { return _dy;}

	protected:
		double _x0;
		double _y0;
		double _dx;
		double _dy;
		int      _rn;
		int      _cn;
	};

}


#endif // MPCDPS_H
