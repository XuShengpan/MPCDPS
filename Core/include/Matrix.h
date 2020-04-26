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

#ifndef MPCDPS_MATRIX_H
#define MPCDPS_MATRIX_H

#include <string>
#include "SmartArrayReal2D.h"
#include "VectorK.h"
#include "VectorX.h"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <fstream>
#include "PublicFunc.h"

namespace mpcdps {

    /* Class Matrix.*/
    template <typename T>
    class Matrix : public SmartArrayReal2D<T>
    {
    public:
        /*Default constructor. */
        Matrix();

        /*Constructor: construct a rn x cn matrix (with the given data if data is not null). */
        Matrix(int rn, int cn, const T* data = NULL);

        /*Copy constructor. */
        Matrix(const Matrix<T>& rth);

        /*Constructor: construct a matrix with a SmartArrayReal2D object. */
        Matrix(const SmartArrayReal2D<T>& rth);

        /*Destructor. */
        ~Matrix();

        /*Clone the matrix. */
        Matrix<T> clone() const;

        //basic operations

        /* ri <-> rj*/
        void row_swap(int ri, int rj);

        /* ri *= t */
        void row_multiply(int ri, T t);

        /* ri += rj*t */
        void row_multiply_add(int ri, int rj, T t);

        /* ci <--> cj*/
        void col_swap(int ci, int cj);

        /* ci *= t */
        void col_multiply(int ci, T t);

        /* ci += cj*t */
        void col_multiply_add(int ci, int cj, T t);

        //mathematical operations

        /* operator += */
        void operator += (const Matrix<T>& rth);

        /* operator -= */
        void operator -= (const Matrix<T>& rth);

        /* operator *= */
        void operator *= (T t);

        /* operator /= */
        void operator /= (T t);

        /* operator + */
        Matrix<T> operator + (const Matrix<T>& rth) const;

        /* operator - */
        Matrix<T> operator - (const Matrix<T>& rth) const;

        /* operator * (with other matrix object). */
        Matrix<T> operator * (const Matrix<T>& rth) const;

        /* operator * (with a scalar). */
        Matrix<T> operator * (T t) const;

        /* operator / */
        Matrix<T> operator / (T t) const;

        //specialized operations

        /* Get the transpose matrix. */
        Matrix<T> transposeMatrix() const;

        /* Get the inverse matrix .
         * A bool type pointer indicates whether the matrix is invertible.
         * If the matrix is not invertible, a empty matrix will be return.
         */
        Matrix<T> inverseMatrix(bool* suc = NULL) const;

        /* Get the rank of the matrix. */
        int rank();

        /* Row simplest for the matrix. */
        void rowSimplest();

        /*Get cofactor of element (i,j). */
        Matrix<T> cofactor(int i, int j) const;

        /*Get norm of the matrix. */
        T norm() const;

        /*Return [A B]. */
        Matrix<T> augmentedMatrix(const Matrix<T>& mat) const;

        /*V1 = M * V, return V1. */
        VectorX<T> operator * (const VectorX<T>& vect) const;

        /*Get minimum eigen value and eigen vector.
         * If failed, return false.
         */
        bool eigenMin(T& eigenvalue, Matrix<T>& eigenvector, T e = 0.0001) const;

        /*Get maximum eigen value and eigen vector.
        * If failed, return false.
        */
        bool eigenMax(T& eigenvalue, Matrix<T>& eigenvector, T e = 0.0001) const;

		/**
		* @brief 求实对称矩阵的特征值及特征向量的雅克比法
		* 利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量
		* @param eigenvalues     特征值数组, 特征值从大到小排列
		* @param eigenvectors    特征向量, 按列存储
		* @param e                    精度要求
		* @param max_iter_time  最大迭代次数
		* @return
		*/
		bool jacbiCor(VectorX<double>& eigenvalues, Matrix<double>& eigenvectors, double e = 0.0001, double max_iter_time = 100) const;

        //static member

        /*return a n-order identity matrix.*/
        static Matrix<T> identityMatrix(int n);

        /**@ Return the 3-order matrix that rotate around x axis by alpha angle.
         * V1 = MV, M : 3x3, V : 3x1
         * alpha: rad
        */
        static Matrix<T> rotate3_x(double alpha);

        /**@ Return the 3-order matrix that rotate around y axis by alpha angle.
        * V1 = MV, M : 3x3, V : 3x1
        * alpha: rad
        */
        static Matrix<T> rotate3_y(double alpha);

        /**@ Return the 3-order matrix that rotate around z axis by alpha angle.
        * V1 = MV, M : 3x3, V : 3x1
        * alpha: rad
        */
        static Matrix<T> rotate3_z(double alpha);

        /**@ Return the 2-order matrix that rotate around z axis by alpha angle.
        * V1 = MV, M : 2x2, V : 2x1
        * alpha: rad
        */
        static Matrix<T> rotate2_z(double alpha);

        /* \brief eigen decompostion for 2x2 matrix
         * \param obj: the input matrix
         * \param eigen_value: eigen values, sort as descending order
         * \param eigen_vector: eigen vectors, place as columns.
        */
        static bool eigen_decomposition_2(const Matrix<T>& obj, Vector2<T>& eigen_value, Matrix<T>& eigen_vector, int& eigen_count);

        //io

        /* Print the matrix to terminate. */
        void print() const;

        /* Output the matrix to the file with precision specified.*/
        void output(const std::string& file, int precision = 6) const;

		void output(std::ofstream& strm) const;

        /* Input the matrix from the give matrix file.*/
        void input(const std::string& matFile);

    protected:
        int _rank; /*The rank of the matrix.*/
    };

    /* M1 = a*M; */
    template <typename T>
    Matrix<T> operator * (double scalar, const Matrix<T>& mat)
    {
        return mat * scalar;
    }

    /* For square matrix:
     *  V1  = MV
     *  nx1 = nxn  nx1 
     */
    template<typename T, int K>
    VectorK<T, K> operator * (const Matrix<T>& mat, const VectorK<T, K>& vect)
    {
        assert(mat.colCount() == K && mat.rowCount() == K);
        VectorK<T, K> v;
        double t = 0;
        for (int r = 0; r < K; ++r, t = 0) {
            for (int c = 0; c < K; ++c) {
                t += mat[r][c] * vect[c];
            }
            v[r] = t;
        }
        return v;
    }

	/* For square matrix:
	*  V1  = V * M
	*  1xn = 1xn nxn
	*/
	template<typename T, int K>
	VectorK<T, K> operator * (const VectorK<T, K>& vect, const Matrix<T>& mat)
	{
		assert(mat.colCount() == K && mat.rowCount() == K);
		VectorK<T, K> v;
		double t = 0;
		for (int c = 0; c < K; ++c) {
			t = 0;
			for (int i = 0; i < K; ++i) {
				t += vect[i] * mat[i][c];
			}
			v[c] = t;
		}
		return v;
	}

    /**@brief
    *  V1 = V*M
    *  1xn = 1xm mxn
    */
    template<typename T>
    VectorX<T> operator*(const VectorX<T>& vect, const Matrix<T>& mat)
    {
        int rn = mat.rowCount(), cn = mat.colCount();
        assert(vect.size() == rn);
        int i = 0;
        VectorX<T> v(cn);
        double t = 0;
        for (int c = 0; c < cn; ++c, t = 0) {
            for (int r = 0; r < rn; ++r, ++i) {
                t += vect[r] * mat[r][c];
            }
            v[c] = t;
        }
        return v;
    }

    /*
     \brief   Transfer vector cross product a x b to S*b or to a*S where a and b are both 3-dim vector and S is a 3 x 3 matrix.
     \return return S.

     for a x b, if vect is a, than transfer to S*b; if vect is b, than transfer to a*S.
    */
    template<typename T>
    Matrix<T> getCrossProductMatrix(const Vector3<T>& vect)
    {
        T data[] = {
            0, -vect[2], vect[1],
            vect[2], 0, -vect[0],
            -vect[1],vect[0], 0
        };
        return Matrix<T>(3, 3, data);
    }
}

#include "Matrix.inl"

#endif
