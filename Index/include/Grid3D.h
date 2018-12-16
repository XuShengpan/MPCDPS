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

#ifndef  MPCDPS_GRID3D_H
#define MPCDPS_GRID3D_H

#include <cmath>
#include <vector>
#include <unordered_map>
#include "PublicInfo.h"

namespace mpcdps {

template<typename T>
class Grid3D
{
public:
    Grid3D()
    {

    }

    ~Grid3D()
    {

    }

    using CellData = std::vector<T>;

    //return the size of the grid
    uint initialize(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, double size)
    {
        _xmin = xmin;
        _ymin = ymin;
        _zmin = zmin;
        _dx = size;
        _cn = (xmax - xmin) / size + 1;
        _rn = (ymax - ymin) / size + 1;
        _hn = (zmax - zmin) / size + 1;

        _grid.clear();
        return _rn * _hn * _cn;
    }

    int get_r(double y) const
    {
        return (y - _ymin) / _dx;
    }

    int get_c(double x) const
    {
        return (x - _xmin) / _dx;
    }

    int get_h(double z) const
    {
        return (z - _zmin) / _dx;
    }

    void push(double* vtx, const T& data)
    {
        uint i = get_i(get_r(vtx[1]), get_c(vtx[0]), get_h(vtx[2]));
        _grid[i].push_back(data);
    }

    bool is_empty(uint i) const
    {
        return _grid.find(i) == _grid.end();
    }

    bool is_empty(int r, int c, int h) const
    {
        return _grid.find(get_i(r, c, h)) == _grid.end();
    }

    CellData get_cell_points(uint i) const
    {
        const auto& iter = _grid.find(i);
        if (iter == _grid.end()) {
            return std::vector<T>();
        } else {
            return iter->second;
        }
    }

    CellData& get_cell_points(uint i)
    {
        return _grid[i];
    }

    CellData get_cell_points(int r, int c, int h) const
    {
        uint i = get_i(r, c, h);
        const auto& iter = _grid.find(i);
        if (iter == _grid.end()) {
            return std::vector<T>();
        } else {
            return iter->second;
        }
    }

    CellData& get_cell_points(int r, int c, int h)
    {
        uint i = get_i(r, c, h);
        return _grid[i];
    }

    void clear()
    {
        _grid.clear();
    }

    int rowCount() const
    {
        return _rn;
    }

    int colCount() const
    {
        return _cn;
    }

    int heightCount() const
    {
        return _hn;
    }

    uint get_i(int r, int c, int h) const
    {
        return c + h * _rn * _cn + r * _cn;
    }

protected:
    double _xmin;
    double _ymin;
    double _zmin;
    uint _rn;
    uint _cn;
    uint _hn;
    double _dx;
    std::unordered_map<uint, CellData> _grid;
};

}

#endif