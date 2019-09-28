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

#ifndef MPCDPS_DETERMINATE_H
#define MPCDPS_DETERMINATE_H

#include "Matrix.h"

namespace mpcdps {

    /* Class Determinate.
     * A determinate has the same row count and column count.
     */
    template <typename T>
    class Determinate: public SmartArrayReal2D<T>
    {
    public:
        /*Default constructor: construct a empty determinate or
         * a n-order determinate and all of the elements will to be set 0. 
         */
        Determinate(int n = 0);

        /*Constructor: construct a determinate from a matrix. */
        Determinate(const Matrix<T>& matrix);

        /*Copy constructor. */
        Determinate(const Determinate<T>& rth);

        /*Destructor. */
        ~Determinate();

        /*Get cofactor of element (i,j). */
        Determinate cofactor(int i, int j) const;

        /*Get algebra cofactor of element (i,j) */
        Determinate algebra_cofactor(int i, int j) const;

        /* ri <-> rj*/
        void row_swap(int ri, int rj);

        /* ri *= t */
        void row_multiply(int ri, T t);

        /* ri += rj*t */
        void row_multiply_add(int ri, int rj, T t);

        /* ci <-> cj*/
        void col_swap(int ci, int cj);

        /* ci *= t */
        void col_multiply(int ci, T t);

        /* ci += cj*t */
        void col_multiply_add(int ci, int cj, T t);

        /* Up-trianglize for the determinate: make up triangle element to 0. */
        void upTrianglize();

        /* Get the value of the determinate. */
        T value();

    private:
        T _factor;  /* factor*/
    };

}

#include "Determinate.inl"

#endif
