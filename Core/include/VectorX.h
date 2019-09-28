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

#ifndef  MPCDPS_VECTORX_H
#define MPCDPS_VECTORX_H

#include "SmartArray.h"

namespace mpcdps {

    /**@brief
    * This class provides a vector of dynamic size.
    */
    template <typename T>
    class  MPCDPS_CORE_ITEM VectorX: public SmartArray<T>
    {
    public:
        /*Constructor.*/
        VectorX();

        /*Construct a vector with n dimension and set all of the dimension data to be v.*/
        VectorX(int n, T v = 0);

        /*Construct a vector with n dimension and initialize the data buffer with data.*/
        VectorX(int n, const T* data);

        /*Copy constructor.*/
        VectorX(const VectorX<T>& obj);

        /*Destructor.*/
        ~VectorX();

        /*Add another vector to this vector.*/
        inline void operator += (const VectorX<T>& obj)
        {
            for (uint i = 0; i < this->_elem_num; ++i)
                this->_data[i] += obj[i];
        }

        /*Subtract another vector from this vector.*/
        inline void operator -= (const VectorX<T>& obj)
        {
            assert(this->_elem_num == obj.size());
            for (uint i = 0; i < this->_elem_num; ++i) {
                this->_data[i] -= obj[i];
            }
        }

        /* Scale the vector with a.*/
        inline void operator *= (double a)
        {
            for (uint i = 0; i < this->_elem_num; ++i) {
                this->_data[i] *= a;
            }
        }

        /* Divide the vector by a.*/
        inline void operator /= (double a)
        {
            for (uint i = 0; i < this->_elem_num; ++i) {
                this->_data[i] /= a;
            }
        }

        /* Get the norm of the vector.*/
        inline T norm() const
        {
            double t = 0;
            for (uint i = 0; i < this->_elem_num; ++i) {
                t += this->_data[i] * this->_data[i];
            }
            return std::sqrt(t);
        }

        /* Get the dot product.*/
        inline T dot(const VectorX<T>& vect) const
        {
            double t = 0;
            for (uint i = 0; i < this->_elem_num; ++i) {
                t += this->_data[i] * vect[i];
            }
            return t;
        }

        /* normalize the vector.*/
        void normalize();

        VectorX<T> operator + (const VectorX<T>& obj) const;
        VectorX<T> operator - (const VectorX<T>& obj) const;
        VectorX<T> operator * (double t) const;
        VectorX<T> operator / (double t) const;
        VectorX<T> operator - () const;
    };

    typedef VectorX<float> VectorXf;
    typedef VectorX<double> VectorXd;

    typedef VectorXd RowVectorXd;
    typedef VectorXd ColumnVectorXd;

}

#endif