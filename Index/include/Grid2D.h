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

#ifndef  MPCDPS_GRID2D_H
#define MPCDPS_GRID2D_H

#include <cmath>
#include "Grid2DFrame.h"
#include "SmartArrayReal2D.h"

namespace mpcdps {

/*
    \note
    dimension 0:  r, related with y
    dimension 1:  c, related with x
*/
template <typename T>
class Grid2D: public SmartArrayReal2D<T>, public Grid2DFrame 
{
public:
    Grid2D()
    {

    }

    Grid2D(const Grid2D& obj)
        : SmartArrayReal2D<T>(obj), Grid2DFrame(obj)
    {

    }

    ~Grid2D()
    {

    }

    typedef T CellType;

    void initialize1(double x_start, double y_start,
        double x_len, double y_len, double dx, double dy)
    {
        Grid2DFrame::initialize1(x_start, y_start, x_len, y_len, dx, dy);
        int rn = Grid2DFrame::rowCount();
        int cn = Grid2DFrame::colCount();
        SmartArrayReal2D<T>* obj = this;
        obj->resize(rn, cn);
    }

    void initialize2(double x_start, double y_start,
        double dx, double dy, int rn, int cn)
    {
        Grid2DFrame::initialize2(x_start, y_start, dx, dy, rn, cn);
        SmartArrayReal2D<T>* obj = this;
        obj->resize(rn, cn);
    }

    int rowCount() const
    {
        return Grid2DFrame::rowCount();
    }

    int colCount() const
    {
        return Grid2DFrame::colCount();
    }
};

}

#endif // MPCDPS_H
