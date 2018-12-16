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
            return this->operator[](r)[c].find(h) == this->operator[](r)[c].end();
        }

        bool get_voxel(int r, int c, int h, VoxelType& voxel) const
        {
            const std::unordered_map<int, VoxelType>& vox = this->operator[](r)[c];
            typename std::unordered_map<int, VoxelType>::const_iterator iter = vox.find(h);
            if (iter != vox.end()) {
                voxel = iter->second;
                return true;
            } else {
                return false;
            }
        }

        std::vector<VoxelIndex> get_neighbor_voxel(const VoxelIndex& index, int neighbhor_depth = 1) const
        {
            std::vector<VoxelIndex> indices;
            if (neighbhor_depth == 0) {
                indices.push_back(index);
                return indices;
            }

            int rn = this->rowCount();
            int cn = this->colCount();
            int hn = heightCount();

            int r_max = index.r + neighbhor_depth + 1;
            int c_max = index.c + neighbhor_depth + 1;
            int h_max = index.h + neighbhor_depth + 1;

            VoxelIndex i_index;
            
            for (int ir = index.r - neighbhor_depth; ir < r_max; ++ir) {
                if (ir < 0) {
                    continue;
                }
                if (ir >= rn) {
                    break;
                }
                for (int ic = index.c - neighbhor_depth; ic < c_max; ++ic) {
                    if (ic < 0) {
                        continue;
                    }
                    if (ic >= cn) {
                        break;
                    }

                    for (int ih = index.h - neighbhor_depth; ih < h_max; ++ih) {
                        if (ih < 0) {
                            continue;
                        }
                        if (ih >= hn) {
                            break;
                        }
                        
                        i_index.r = ir;
                        i_index.c = ic;
                        i_index.h = ih;

                        indices.push_back(i_index);
                    }
                }
            }

            return indices;
        }

    protected:
        double _z0;
        double _dz;
        int _hn;
    };

    class Voxel
    {
    public:
        Voxel() {}
        ~Voxel() {}

        /*
        void add_point(int ptid);
        */
        virtual void add_point(int ptid)
        {
            _ptids.push_back(ptid);
        }

        std::vector<int> get_ptids() const
        {
            return _ptids;
        }

        size_t size() const
        {
            return _ptids.size();
        }

    protected:
        std::vector<int> _ptids;
    };
}

#endif