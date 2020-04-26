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

#ifndef  MPCDPS_GRID3D_H
#define MPCDPS_GRID3D_H

#include <unordered_map>
#include <string>
#include <sstream>
#include <VectorK.h>

namespace mpcdps {

template<typename CellType>
class Grid3D
{
public:
    Grid3D()
    {
    }

    ~Grid3D()
    {

    }

    typedef Vector3<int> Index;

    void seResolution(float resolution)
    {
        _resolution = resolution;
    }

    std::string getIndexKey(float x, float y, float z) const
    {
        float ix = x / _resolution;
        float iy = y / _resolution;
        float iz = z / _resolution;
        return get_string(ix) + "_" + get_string(iy) + "_" + get_string(iz);
    }

    CellType& getCell(const std::string& key)
    {
        return _data[key];
    }

    CellType getCell(const std::string& key) const
    {
        auto iter = _data.find(key);
        return iter->second;
    }

    const auto& container() const
    {
        return _data;
    }

protected:
    std::string get_string(float v) const
    {
        int n = v;
        std::ostringstream strm;
        strm << n;
        std::string str = strm.str();
        if (n == 0 && v < 0) {
            str = "-" + str;
        }
        return str;
    }

protected:
    float _resolution = 0;
    std::unordered_map<std::string, CellType> _data;
};

}

#endif