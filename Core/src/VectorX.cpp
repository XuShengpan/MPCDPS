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

#include "VectorX.h"
#include <cmath>

namespace mpcdps {

    template <typename T>
    VectorX<T>::VectorX()
    {
    }

    template <typename T>
    VectorX<T>::VectorX(int n, T v): SmartArray<T>(n)
    {
        this->reset(v);
    }

    template <typename T>
    VectorX<T>::VectorX(int n, const T* data):SmartArray<T>(n)
    {
        for (int i = 0; i < n; ++i) {
            this->_data[i] = data[i];
        }
    }

    template <typename T>
    VectorX<T>::VectorX(const VectorX<T>& obj):SmartArray<T>(obj)
    {
    }

    template <typename T>
    VectorX<T>::~VectorX()
    {
    }

    template <typename T>
    VectorX<T> VectorX<T>::operator + (const VectorX<T>& obj) const
    {
        uint sz = this->_elem_num;
        assert(sz == obj.size());
        VectorX<T> v(sz);
        for (uint i = 0; i < sz; ++i) {
            v[i] = this->_data[i] + obj[i];
        }
        return v;
    }

    template <typename T>
    VectorX<T> VectorX<T>::operator - (const VectorX<T>& obj) const
    {
        uint sz = this->_elem_num;
        assert(sz == obj.size());
        VectorX<T> v(sz);
        for (uint i = 0; i < sz; ++i) {
            v[i] = this->_data[i] - obj[i];
        }
        return v;
    }

    template <typename T>
    VectorX<T> VectorX<T>::operator * (double t) const
    {
        VectorX<T> v(this->_elem_num);
        for (uint i = 0; i < this->_elem_num; ++i) {
            v[i] = this->_data[i]*t;
        }
        return v;
    }

    template <typename T>
    VectorX<T> VectorX<T>::operator / (double t) const
    {
        VectorX<T> v(this->_elem_num);
        for (uint i = 0; i < this->_elem_num; ++i) {
            v[i] = this->_data[i]/t;
        }
        return v;
    }

    template <typename T>
    VectorX<T> VectorX<T>::operator - () const
    {
        VectorX<T> v(this->_elem_num);
        for (uint i = 0; i < this->_elem_num; ++i) {
            v[i] = -this->_data[i];
        }
        return v;
    }

    template <typename T>
    void VectorX<T>::normalize()
    {
        double t = norm();
        if (t > ZERO_D) {
            for (uint i = 0; i < this->_elem_num; ++i) {
                this->_data[i] /= t;
            }
        }
    }

	template MPCDPS_CORE_ITEM class VectorX<int>;
    template MPCDPS_CORE_ITEM class VectorX<float>;
    template MPCDPS_CORE_ITEM class VectorX<double>;
}