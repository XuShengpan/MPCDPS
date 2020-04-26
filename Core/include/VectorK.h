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

#ifndef  MPCDPS_VECTORK_H
#define MPCDPS_VECTORK_H

#include <vector>
#include "Tuple.h"

namespace mpcdps {

    /* This class provides a vector of fixed size. */
    template <typename T, int K>
    class  VectorK: public Tuple<T, K>
    {
    public:

        /*Constructor.*/
        VectorK()
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] = 0;
            }
        }

        /*Constructor.*/
        VectorK(const float* vect)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] = vect[i];
            }
        }

        /*Constructor.*/
        VectorK(const double* vect)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] = vect[i];
            }
        }

        /*Copy constructor.*/
        VectorK(const VectorK<T, K>& vect)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] = vect[i];
            }
        }

        /*Add another vector to this vector.*/
        void operator += (const VectorK<T, K>& obj)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] += obj[i];
            }
        }

        /*Subtract another vector from this vector.*/
        void operator -= (const VectorK<T, K>& obj)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] -= obj[i];
            }
        }

        /* Scale the vector with a.*/
        void operator *= (double a)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] *= a;
            }
        }

        /* Divide the vector by a.*/
        void operator /= (double a)
        {
            for (int i = 0; i < K; ++i) {
                this->_data[i] /= a;
            }
        }

        /* Get the norm of the vector.*/
        T norm() const
        {
            double t = 0;
            for (int i = 0; i < K; ++i) {
                t += this->_data[i] * this->_data[i];
            }
            return std::sqrt(t);
        }

        /* Get the dot product.*/
        T dot(const VectorK<T, K>& vect) const
        {
            double t = 0;
            for (int i = 0; i < K; ++i) {
                t += this->_data[i] * vect[i];
            }
            return t;
        }

        /* normalize the vector.*/
        void normalize()
        {
            T t = norm();
            if (t > ZERO_D) {
                for (int i = 0; i < K; ++i) {
                    this->_data[i] /= t;
                }
            }
        }

        /*Add another vector, return the result vector.*/
        VectorK<T, K> operator + (const VectorK<T, K>& obj) const
        {
            VectorK<T, K> vect;
            for (int i = 0; i < K; ++i) {
                vect[i] = this->_data[i] + obj[i];
            }
            return vect;
        }

        /*Minus another vector, return the result vector.*/
        VectorK<T, K> operator - (const VectorK<T, K>& obj) const
        {
            VectorK<T, K> vect;
            for (int i = 0; i < K; ++i) {
                vect[i] = this->_data[i] - obj[i];
            }
            return vect;
        }

        /*Return the negative vector.*/
        VectorK<T, K> operator - () const
        {
            VectorK<T, K> vect;
            for (int i = 0; i < K; ++i) {
                vect[i] = -this->_data[i];
            }
            return vect;
        }

        /*Return the vector scaled by t.*/
        VectorK<T, K> operator * (double t) const
        {
            VectorK<T, K> vect;
            for (int i = 0; i < K; ++i) {
                vect[i] = this->_data[i] * t;
            }
            return vect;
        }

		/*Return the vector divided by t.*/
		VectorK<T, K> operator / (double t) const
		{
			VectorK<T, K> vect;
			for (int i = 0; i < K; ++i) {
				vect[i] = this->_data[i] / t;
			}
			return vect;
		}
    };

    /* Return the vector of a constant number times a vector.*/
    template <typename T, int K>
    VectorK<T, K> operator * (double a, const VectorK<T, K>& vect)
    {
        VectorK<T, K> vect1;
        for (int i = 0; i < K; ++i) {
            vect1[i] = vect[i] * a;
        }
        return vect1;
    }


    /* Two dimension vector.*/
    template <typename T>
    class Vector2: public VectorK<T, 2>
    {
    public:
        /* Default constructor.*/
        Vector2()
        {
        }

        /* Construct a Vector2 with (x, y)*/
        Vector2(T x, T y)
        {
            this->_data[0] = x;
            this->_data[1] = y;
        }

        /* Copy constructor.*/
        Vector2(const VectorK<T, 2>& obj)
        {
            this->_data[0] = obj[0];
            this->_data[1] = obj[1];
        }

        /* Return x.*/
        T x() const {return this->_data[0];}

        /*Return y.*/
        T y() const {return this->_data[1];}

        /*Return the determinate with another Vector2 vector.*/
        double determinate(const Vector2<T>& vect) const
        {
            return this->_data[0]*vect[1] - this->_data[1]*vect[0];
        }
    };

    /* Three dimension vector.*/
    template <typename T>
    class Vector3: public VectorK<T, 3>
    {
    public:
        /* Default constructor.*/
        Vector3()
        {

        }

        /* Construct a Vector3 with (x, y, z)*/
        Vector3(T x, T y, T z)
        {
            this->_data[0] = x;
            this->_data[1] = y;
            this->_data[2] = z;
        }

        /* Copy constructor.*/
        Vector3(const VectorK<T, 3>& obj)
        {
            this->_data[0] = obj[0];
            this->_data[1] = obj[1];
            this->_data[2] = obj[2];
        }

        T x() const {return this->_data[0];}
        T y() const {return this->_data[1];}
        T z() const {return this->_data[2];}

        /* Return cross product with another Vector3.*/
        Vector3<T> cross(const Vector3<T>& vect) const
        {
            Vector3<T> vect1;
            vect1[0] = this->_data[1] * vect[2] - vect[1] * this->_data[2];
            vect1[1] = this->_data[2] * vect[0] - vect[2] * this->_data[0];
            vect1[2] = this->_data[0] * vect[1] - vect[0] * this->_data[1];
            return vect1;
        }
    };

    /* Four dimension vector.*/
    template <typename T>
    class Vector4: public VectorK<T, 4>
    {
    public:
        /* Default constructor.*/
        Vector4()
        {

        }

        /* Construct a Vector3 with (x, y, z, w)*/
        Vector4(T x, T y, T z, T w)
        {
            this->_data[0] = x;
            this->_data[1] = y;
            this->_data[2] = z;
            this->_data[3] = w;
        }

        /* Copy constructor.*/
        Vector4(const VectorK< T, 4>& obj)
        {
            this->_data[0] = obj[0];
            this->_data[1] = obj[1];
            this->_data[2] = obj[2];
            this->_data[3] = obj[3];
        }

        T x() const {return this->_data[0];}
        T y() const {return this->_data[1];}
        T z() const {return this->_data[2];}
        T w() const {return this->_data[3];}
    };

    typedef Vector2< float> Vector2f;
    typedef Vector2< double> Vector2d;
    
    typedef Vector3< float> Vector3f;
    typedef Vector3< double> Vector3d;

    typedef Vector4< float> Vector4f;
    typedef Vector4< double> Vector4d;

}

#endif
