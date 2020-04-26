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

#ifndef  MPCDPS_VOXELGRID_H
#define MPCDPS_VOXELGRID_H

#include <Grid2D.h>
#include <unordered_map>

namespace mpcdps {

    /*
       \note
       dimension 0:  r, related with y
       dimension 1:  c, related with x
       dimension 2:  h, related with z
    */
    template<typename VoxelType>
    class VoxelGrid: public Grid2D<std::unordered_map<int, VoxelType> >
    {
    public:
        VoxelGrid()
        {
        }

        ~VoxelGrid()
        {
        }

        typedef VoxelType VoxelT;

        struct VoxelIndex
        {
            int r;
            int c;
            int h;
        };

        void initialize_z(double zmin, double z_len, double dz)
        {
            _z0 = zmin;
            _dz = dz;
            _hn = std::ceil(z_len + 0.001) / dz;
        }

        int get_h(double z) const
        {
            return int((z - _z0) / _dz);
        }

        int heightCount() const
        {
            return _hn;
        }

        double get_dz() const
        {
            return _dz;
        }

        double get_zStart() const
        {
            return _z0;
        }

        double get_z(int h) const
        {
            return SKIP(_z0, _dz, h);
        }

        bool is_empty(int r, int c, int h) const
        {
            return (*this)[r][c].count(h) == 0;
        }

    protected:
        double _z0;
        double _dz;
        int _hn;
    };
}

#endif