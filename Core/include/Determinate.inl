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

namespace mpcdps {

    template <typename T>
    Determinate<T>::Determinate(int n):SmartArrayReal2D<T>(n, n), _factor(1)
    {
        if (n > 0) {
            memset(this->_data, 0, n * n * sizeof(T));
        }
    }

    template <typename T>
    Determinate<T>::Determinate(const Matrix<T>& matrix)
        :SmartArrayReal2D<T>(matrix.rowCount(), matrix.colCount(), matrix.buffer()), _factor(1)
    {
        assert(this->_rn == this->_cn);
    }

    template <typename T>
    Determinate<T>::Determinate(const Determinate<T>& rth):
        SmartArrayReal2D<T>(rth._rn, rth._cn, rth._data), _factor(rth._factor)
    {

    }

    template <typename T>
    Determinate<T>::~Determinate()
    {

    }

    template <typename T>
    Determinate<T> Determinate<T>::cofactor(int i, int j) const
    {
        Determinate<T> cof(this->_rn - 1);
        int r1 = 0;
        int c1 = 0;
        for (int r = 0; r < this->_rn; ++r) {
            if (r == i) {
                continue;
            }
            c1 = 0;
            for (int c = 0; c < this->_cn; ++c) {
                if (c == j) {
                    continue;
                }
                cof[r1][c1] = this->_data[this->linearId(r, c)];
                ++c1;
            }
            ++r1;
        }
        return cof;
    }

    template <typename T>
    Determinate<T> Determinate<T>::algebra_cofactor(int i, int j) const
    {
        Determinate<T> cof = cofactor(i, j);
        if ((i + j) % 2) {
            cof._factor = -1;
        }
        return cof;
    }

    //basic operations
    //ri <--> rj
    template <typename T>
    void Determinate<T>::row_swap(int ri, int rj)
    {
        T t;
        for (int k = 0; k < this->_cn; ++k) {
            t = this->_data[this->linearId(ri, k)];
            this->_data[this->linearId(ri, k)] = this->_data[this->linearId(rj, k)];
            this->_data[this->linearId(rj, k)] = t;
        }
        _factor = -_factor;
    }

    //ri *= t
    template <typename T>
    void Determinate<T>::row_multiply(int ri, T t)
    {
        for (int k = 0; k < this->_cn; ++k) {
            this->_data[this->linearId(ri, k)] *= t;
        }
        _factor /= t;
    }

    //ri += rj*t
    template <typename T>
    void Determinate<T>::row_multiply_add(int ri, int rj, T t)
    {
        for (int k = 0; k < this->_cn; ++k) {
            this->_data[this->linearId(ri, k)] += this->_data[this->linearId(rj, k)] * t;
        }
    }

    //ci <--> cj
    template <typename T>
    void Determinate<T>::col_swap(int ci, int cj)
    {
        T t;
        for (int k = 0; k < this->_rn; ++k) {
            t = this->_data[this->linearId(k, ci)];
            this->_data[this->linearId(k, ci)] = this->_data[this->linearId(k, cj)];
            this->_data[this->linearId(k, cj)] = t;
        }
        _factor = -_factor;
    }

    //ci *= t
    template <typename T>
    void Determinate<T>::col_multiply(int ci, T t)
    {
        for (int k = 0; k < this->_rn; ++k) {
            this->_data[this->linearId(k, ci)] *= t;
        }
        _factor /= t;
    }

    //ci += cj*t
    template <typename T>
    void Determinate<T>::col_multiply_add(int ci, int cj, T t)
    {
        for (int k = 0; k < this->_rn; ++k) {
            this->_data[this->linearId(k, ci)] += this->_data[this->linearId(k, cj)] * t;
        }
    }

    template <typename T>
    void Determinate<T>::upTrianglize()
    {
        for (int r = 0; r < this->_rn; ++r) {
            for (int c = 0; c < r; ++c) {
                if (std::abs(this->_data[this->linearId(r, c)]) > ZERO_D) {
                    if (std::abs(this->_data[this->linearId(c, c)]) <= ZERO_D) {
                        row_swap(r, c);
                    } else {
                        row_multiply_add(r, c, 
                            -this->_data[this->linearId(r, c)] / this->_data[this->linearId(c, c)]);
                    }
                }
            }
        }
    }

    template <typename T>
    T Determinate<T>::value()
    {
        upTrianglize();
        T d = _factor;
        for (int i = 0; i < this->_rn; ++i) {
            d *= this->_data[this->linearId(i, i)];
        }
        return d;
    }
}
