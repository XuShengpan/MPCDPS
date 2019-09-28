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

namespace mpcdps {

    template<typename T>
    Matrix<T>::Matrix():_rank(-1)
    {
    }

    template<typename T>
    Matrix<T>::Matrix(int rn, int cn, const T* data):SmartArrayReal2D<T>(rn, cn, data), _rank(-1)
    {
        if (!data) {
            int n = rn * cn;
            memset(this->_data, 0, n * sizeof(T));
        }
    }

    template<typename T>
    Matrix<T>::Matrix(const Matrix<T>& rth): 
        SmartArrayReal2D<T>(rth._rn, rth._cn, rth._data), _rank(rth._rank)
    {
    }

    template<typename T>
    Matrix<T>::Matrix(const SmartArrayReal2D<T>& rth): SmartArrayReal2D<T>(rth), _rank(-1)
    {
    }

    template<typename T>
    Matrix<T>::~Matrix()
    {
    }

    template<typename T>
    Matrix<T> Matrix<T>::clone() const
    {
        Matrix<T> rth = SmartArrayReal2D<T>::clone();
        rth._rank = _rank;
        return rth;
    }

    //basic operations
    //ri <--> rj
    template<typename T>
    void Matrix<T>::row_swap(int ri, int rj)
    {
        T t;
        for (int k = 0; k < this->_cn; ++k) {
            t = this->_data[this->linearId(ri, k)];
            this->_data[this->linearId(ri, k)] = this->_data[this->linearId(rj, k)];
            this->_data[this->linearId(rj, k)] = t;
        }
    }

    //ri *= t
    template<typename T>
    void Matrix<T>::row_multiply(int ri, T t)
    {
        for (int k = 0; k < this->_cn; ++k) {
            this->_data[this->linearId(ri, k)] *= t;
        }
    }

    //ri += rj*t
    template<typename T>
    void Matrix<T>::row_multiply_add(int ri, int rj, T t)
    {
        for (int k = 0; k < this->_cn; ++k) {
            this->_data[this->linearId(ri, k)] += this->_data[this->linearId(rj, k)] * t;
        }
    }

    //ci <--> cj
    template<typename T>
    void Matrix<T>::col_swap(int ci, int cj)
    {
        T t;
        for (int k = 0; k < this->_rn; ++k) {
            t = this->_data[this->linearId(k, ci)];
            this->_data[this->linearId(k, ci)] = this->_data[this->linearId(k, cj)];
            this->_data[this->linearId(k, cj)] = t;
        }
    }

    //ci *= t
    template<typename T>
    void Matrix<T>::col_multiply(int ci, T t)
    {
        for (int k = 0; k < this->_rn; ++k) {
            this->_data[this->linearId(k, ci)] *= t;
        }
    }

    //ci += cj*t
    template<typename T>
    void Matrix<T>::col_multiply_add(int ci, int cj, T t)
    {
        for (int k = 0; k < this->_rn; ++k) {
            this->_data[this->linearId(k, ci)] += this->_data[this->linearId(k, cj)] * t;
        }
    }

    //mathematical operations
    template<typename T>
    Matrix<T> Matrix<T>::operator + (const Matrix<T>& rth) const
    {
        int rn = this->_rn;
        int cn = this->_cn;
        assert(rn == rth._rn && cn == rth._cn);
        Matrix<T> matr(rn, cn);
        T* buf = matr.buffer();
        T* buf1 = rth.buffer();
        int n = rn * cn;
        for (int i = 0; i < n; ++i) {
            buf[i] = this->_data[i] + buf1[i];
        }
        return matr;
    }

    template<typename T>
    Matrix<T> Matrix<T>::operator - (const Matrix<T>& rth) const
    {
        int rn = this->_rn;
        int cn = this->_cn;
        assert(rn == rth._rn && cn == rth._cn);
        Matrix<T> matr(rn, cn);
        T* buf = matr.buffer();
        T* buf1 = rth.buffer();
        int n = rn * cn;
        for (int i = 0; i < n; ++i) {
            buf[i] = this->_data[i] - buf1[i];
        }
        return matr;
    }

    template<typename T>
    Matrix<T> Matrix<T>::operator * (const Matrix<T>& rth) const
    {
        int rn = this->rowCount();
        int k = this->colCount();
        int cn = rth._cn;
        assert(k == rth._rn);
        Matrix<T> mat(rn, cn);
        double t = 0;
        int i0 = 0;
        int i = 0;
        for (int r = 0; r < rn; ++r, i0 += k) {
            for (int c = 0; c < cn; ++c) {
                t = 0;
                i = i0;
                for (int ki = 0; ki < k; ++ki, ++i) {
                    t += this->_data[i] * rth[ki][c];
                }
                mat[r][c] = t;
            }
        }
        return mat;
    }

    template<typename T>
    Matrix<T> Matrix<T>::operator * (T t) const
    {
        int rn = this->_rn;
        int cn = this->_cn;
        Matrix<T> matr(rn, cn);
        T* buf = matr.buffer();
        int n = rn * cn;
        for (int i = 0; i < n; ++i) {
            buf[i] = this->_data[i] * t;
        }
        return matr;
    }

    template<typename T>
    Matrix<T> Matrix<T>::operator / (T t) const
    {
        int rn = this->_rn;
        int cn = this->_cn;
        Matrix<T> matr(rn, cn);
        T* buf = matr.buffer();
        int n = rn * cn;
        for (int i = 0; i < n; ++i) {
            buf[i] = this->_data[i] / t;
        }
        return matr;
    }

    template<typename T>
    void Matrix<T>::operator += (const Matrix<T>& rth)
    {
        assert(this->_rn == rth._rn && this->_cn == rth._cn);
        int n = this->_rn * this->_cn;
        for (int i = 0; i < n; ++i) {
            this->_data[i] += rth._data[i];
        }
    }

    template<typename T>
    void Matrix<T>::operator -= (const Matrix<T>& rth)
    {
        assert(this->_rn == rth._rn && this->_cn == rth._cn);
        int n = this->_rn * this->_cn;
        for (int i = 0; i < n; ++i) {
            this->_data[i] -= rth._data[i];
        }
    }

    template<typename T>
    void Matrix<T>::operator *= (T t)
    {
        int n = this->_rn * this->_cn;
        for (int i = 0; i < n; ++i) {
            this->_data[i] *= t;
        }
    }

    template<typename T>
    void Matrix<T>::operator /= (T t)
    {
        int n = this->_rn * this->_cn;
        for (int i = 0; i < n; ++i) {
            this->_data[i] /= t;
        }
    }

    //specialized operations
    template<typename T>
    Matrix<T> Matrix<T>::transposeMatrix() const
    {
        int rn = this->_rn;
        int cn = this->_cn;
        Matrix<T> matr(cn, rn);
        for (int r = 0; r < cn; ++r) {
            for (int c = 0; c < rn; ++c) {
                matr[r][c] = this->_data[this->linearId(c, r)];
            }
        }
        return matr;
    }

    template<typename T>
    Matrix<T> Matrix<T>::inverseMatrix(bool* suc) const
    {
        if (suc) {
            *suc = false;
        }

        if (this->_rn != this->_cn) {
            Matrix<T>();
        }
        int n = this->_rn;
        Matrix<T> augm = augmentedMatrix(identityMatrix(n));
        augm.rowSimplest();
        if (std::abs(augm[n-1][n-1]) < ZERO_F) {
            return Matrix<T>();
        }

        if (suc) {
            *suc = true;
        }
        return augm.block(0, n, n, n);
    }

    template<typename T>
    int Matrix<T>::rank()
    {
        if (_rank != -1) {
            return _rank;
        }
        rowSimplest();
        return _rank;
    }

    template<typename T>
    void Matrix<T>::rowSimplest()
    {
        T* data = this->_data;
        int r = 0;
        int c = 0;
        int rk = 0;
        while (r < this->_rn && c < this->_cn) {
            int r1 = r;
            while (r1 < this->_rn && std::abs(data[this->linearId(r1, c)]) <= ZERO_D) {
                ++r1;
            }

            if (r1 == this->_rn) {
                ++c;
            } else {
                if (r1 > r) {
                    row_swap(r, r1);
                }
                row_multiply(r, 1.0 / data[this->linearId(r, c)]);
                for (r1 = 0; r1 < this->_rn; ++r1) {
                    if (r1 == r || std::abs(data[this->linearId(r1, c)]) <= ZERO_D) {
                        continue;
                    }
                    row_multiply_add(r1, r, -data[this->linearId(r1, c)]);
                }
            ++rk;
            ++r;
            ++c;
            }
        }
        _rank = rk;
    }

    template<typename T>
    Matrix<T> Matrix<T>::cofactor(int i, int j) const
    {
        Matrix<T> cof(this->_rn - 1, this->_cn - 1);
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

    template<typename T>
    T Matrix<T>::norm() const
    {
        T d = 0;
        int n = this->_rn*this->_cn;
        for (int i = 0; i < n; ++i) {
            d += this->_data[i] * this->_data[i];
        }
        return std::sqrt(d);
    }

    //A.aug(B) =[A B]
    template<typename T>
    Matrix<T> Matrix<T>::augmentedMatrix(const Matrix<T>& mat) const
    {
        assert(this->_rn == mat._rn);
        int cn = this->_cn + mat._cn;
        Matrix<T> aug(this->_rn, cn);
        int c;
        for (int r = 0; r < this->_rn; ++r) {
            for (c = 0; c < this->_cn; ++c) {
                aug[r][c] = this->_data[this->linearId(r, c)];
            }
            for (int c1 = 0; c1 < mat._cn; ++c1, ++c) {
                aug[r][c] = mat[r][c1];
            }
        }
        return aug;
    }

    template<typename T>
    VectorX<T> Matrix<T>::operator * (const VectorX<T>& vect) const
    {
        int rn = this->rowCount(), cn = this->colCount();
        assert(cn == vect.size());
        int i = 0;
        T* data = this->_data;
        VectorX<T> v(rn);
        double t = 0;
        for (int r = 0; r < rn; ++r, t = 0) {
            for (int c = 0; c < cn; ++c, ++i) {
                t += data[i] * vect[c];
            }
            v[r] = t;
        }
        return v;
    }

    template<typename T>
    void power_iteration(const Matrix<T>& inputMatrix, T e, T& eigenvalue, Matrix<T>& eigenvector)
    {
        Matrix<T> u(inputMatrix.rowCount(), 1);
        Matrix<T> y;
        Matrix<T> y_trans;
        T* data_in = u.buffer();
        data_in[0] = 1;
        y = u;
        u = inputMatrix*y;
        T b0;
        T b;
        Matrix<T> m1;
        Matrix<T> m2;
        y_trans = y.transposeMatrix();
        m1 = y_trans * u;
        m2 = y_trans * y;
        b = m1[0][0] / m2[0][0];
        int n = 0;
        do
        {
            y = u / u.norm();
            u = inputMatrix * y;
            y_trans = y.transposeMatrix();
            b0 = b;
            m1 = y_trans * u;
            m2 = y_trans * y;
            b = m1[0][0] / m2[0][0];
            ++n;

        } while (n < 30 && std::abs((b - b0) / b) > e);

        eigenvalue = b;
        eigenvector = y;
    }

    template<typename T>
    bool Matrix<T>::eigenMin(T& eigenvalue, Matrix<T>& eigenvector, T e) const
    {
        if (this->_rn != this->_cn) {
            return false;
        }
        bool tag;
        Matrix inver = inverseMatrix(&tag);
        if (!tag) {
            return false;
        }
        T b;
        power_iteration(inver, e, b, eigenvector);
        eigenvalue = 1 / b;
        return true;
    }

    template<typename T>
    bool Matrix<T>::eigenMax(T& eigenvalue, Matrix<T>& eigenvector, T e) const
    {
        if (this->_rn != this->_cn) {
            return false;
        }
        Matrix<T> m1 = clone();
        if (m1.rank() < m1.rowCount()) {
            return false;
        }
        power_iteration(*this, e, eigenvalue, eigenvector);
        return true;
    }

    //member
    template<typename T>
    Matrix<T> Matrix<T>::identityMatrix(int n)
    {
        Matrix<T> mat(n, n);
        for (int i = 0; i < n; ++i) {
            mat[i][i] = 1;
        }
        return mat; 
    }

    /*
    alpha: rad
    V1 = MV, M: 3x3, V: 3x1
    */
    template<typename T>
    Matrix<T> Matrix<T>::rotate3_x(double alpha)
    {
        T data[] = {
            1, 0, 0,
            0, T(std::cos(alpha)), T(-std::sin(alpha)),
            0, T(std::sin(alpha)), T(std::cos(alpha))
        };
        return Matrix<T>(3, 3, data);
    }

    /*
    alpha: rad
    V1 = MV, M: 3x3, V: 3x1
    */
    template<typename T>
    Matrix<T> Matrix<T>::rotate3_y(double alpha)
    {
        T data[] = {
            T(std::cos(alpha)), 0, T(std::sin(alpha)),
            0, 1, 0,
            T(-std::sin(alpha)), 0, T(std::cos(alpha))
        };
        return Matrix<T>(3, 3, data);
    }

    /*
    alpha: rad
    V1 = MV, M: 3x3, V: 3x1
    */
    template<typename T>
    Matrix<T> Matrix<T>::rotate3_z(double alpha)
    {
        T data[] = {
            T(std::cos(alpha)), T(-std::sin(alpha)), 0,
            T(std::sin(alpha)), T(std::cos(alpha)), 0,
            0, 0, 1
        };
        return Matrix<T>(3, 3, data);
    }

    /*
    alpha: rad
    V1 = MV, M: 2x2, V: 2x1
    */
    template<typename T>
    Matrix<T> Matrix<T>::rotate2_z(double alpha)
    {
        T data[] = {
            T(std::cos(alpha)), T(-std::sin(alpha)),
            T(std::sin(alpha)), T(std::cos(alpha))
        };
        return Matrix<T>(2, 2, data);
    }

    //io
    template<typename T>
    void Matrix<T>::print() const
    {
        std::setiosflags(std::ios::fixed);
        std::setiosflags(std::ios::left);

        for (int r = 0; r < this->_rn; ++r) {
            for (int c = 0; c < this->_cn; ++c) {
                std::cout << this->_data[this->linearId(r, c)] << "  ";
            }
            std::cout << std::endl;
        }
    }

    template<typename T>
    void Matrix<T>::output(const std::string& matFile, int precision) const
    {
        std::string str = matFile;
        std::ofstream outfile(str.c_str());
        outfile << this->_rn << "   " << this->_cn << std::endl;

        outfile.precision(precision);
        outfile.setf(std::ios::fixed | std::ios::left);

        int i = 0;
        for (int r = 0; r < this->_rn; ++r) {
            for (int c = 0; c < this->_cn; ++c, ++i) {
                outfile << this->_data[i] << "  ";
            }
            outfile << std::endl;
        }
    }

	template<typename T>
	void Matrix<T>::output(std::ofstream& strm) const
	{
		int i = 0;
		for (int r = 0; r < this->_rn; ++r) {
			for (int c = 0; c < this->_cn; ++c, ++i) {
				strm << this->_data[i] << "  ";
			}
			strm << std::endl;
		}
	}

    template<typename T>
    void Matrix<T>::input(const std::string& matFile)
    {
        std::string cstr = matFile;

        std::ifstream infile(cstr.c_str());
        int rn;
        int cn;
        infile >> rn >> cn;
        this->clear();
        this->resize(rn, cn);
        int i = 0;
        T t;
        for (int r = 0; r < rn; ++r) {
            for (int c = 0; c < cn; ++c, ++i) {
                infile >> t;
                this->_data[i] = t;
            }
        }
    }

	//http://blog.csdn.net/zhouxuguang236/article/details/40212143
	template<typename T>
	bool Matrix<T>::jacbiCor(VectorX<double>& eigenvalues, Matrix<double>& eigenvectors, double eps, double max_iter_time) const
	{
		int nDim = this->rowCount();
		assert(this->colCount() == nDim);

		Matrix<T> obj = this->clone();
		T * pMatrix = obj.buffer();
		eigenvalues.resize(nDim);
		eigenvectors.resize(nDim, nDim);

		double *pdblVects = eigenvectors.buffer();
		double* pdbEigenValues = eigenvalues.buffer();
		double dbEps = eps;
		double nJt = max_iter_time;

		for (int i = 0; i < nDim; i++) {
			pdblVects[i*nDim + i] = 1.0f;
			for (int j = 0; j < nDim; j++) {
				if (i != j)
					pdblVects[i*nDim + j] = 0.0f;
			}
		}

		int nCount = 0;     //迭代次数  
		while (1) {
			//在pMatrix的非对角线上找到最大元素  
			double dbMax = pMatrix[1];
			int nRow = 0;
			int nCol = 1;
			for (int i = 0; i < nDim; i++)          //行  
			{
				for (int j = 0; j < nDim; j++)      //列  
				{
					double d = std::abs(pMatrix[i*nDim + j]);
					if ((i != j) && (d > dbMax)) {
						dbMax = d;
						nRow = i;
						nCol = j;
					}
				}
			}

			if (dbMax < dbEps)     //精度符合要求   
				break;

			if (nCount > nJt)       //迭代次数超过限制  
				break;

			nCount++;

			double dbApp = pMatrix[nRow*nDim + nRow];
			double dbApq = pMatrix[nRow*nDim + nCol];
			double dbAqq = pMatrix[nCol*nDim + nCol];

			//计算旋转角度  
			double dbAngle = 0.5*atan2(-2 * dbApq, dbAqq - dbApp);
			double dbSinTheta = sin(dbAngle);
			double dbCosTheta = cos(dbAngle);
			double dbSin2Theta = sin(2 * dbAngle);
			double dbCos2Theta = cos(2 * dbAngle);

			pMatrix[nRow*nDim + nRow] = dbApp*dbCosTheta*dbCosTheta +
				dbAqq*dbSinTheta*dbSinTheta + 2 * dbApq*dbCosTheta*dbSinTheta;
			pMatrix[nCol*nDim + nCol] = dbApp*dbSinTheta*dbSinTheta +
				dbAqq*dbCosTheta*dbCosTheta - 2 * dbApq*dbCosTheta*dbSinTheta;
			pMatrix[nRow*nDim + nCol] = 0.5*(dbAqq - dbApp)*dbSin2Theta + dbApq*dbCos2Theta;
			pMatrix[nCol*nDim + nRow] = pMatrix[nRow*nDim + nCol];

			for (int i = 0; i < nDim; i++) {
				if ((i != nCol) && (i != nRow)) {
					int u = i*nDim + nRow;  //p    
					int w = i*nDim + nCol;  //q  
					dbMax = pMatrix[u];
					pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax*dbCosTheta;
					pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax*dbSinTheta;
				}
			}

			for (int j = 0; j < nDim; j++) {
				if ((j != nCol) && (j != nRow)) {
					int u = nRow*nDim + j;  //p  
					int w = nCol*nDim + j;  //q  
					dbMax = pMatrix[u];
					pMatrix[u] = pMatrix[w] * dbSinTheta + dbMax*dbCosTheta;
					pMatrix[w] = pMatrix[w] * dbCosTheta - dbMax*dbSinTheta;
				}
			}

			//计算特征向量  
			for (int i = 0; i < nDim; i++) {
				int u = i*nDim + nRow;      //p     
				int w = i*nDim + nCol;      //q  
				dbMax = pdblVects[u];
				pdblVects[u] = pdblVects[w] * dbSinTheta + dbMax*dbCosTheta;
				pdblVects[w] = pdblVects[w] * dbCosTheta - dbMax*dbSinTheta;
			}
		}

		{
			//对特征值进行排序以及重新排列特征向量,特征值即pMatrix主对角线上的元素

			std::vector<double> ev(nDim);
			for (int i = 0; i < nDim; ++i) {
				ev[i] = pMatrix[i*nDim + i];
			}

			Matrix<double> evc = eigenvectors.clone();

			std::vector<int> ii = make_vector<int>(nDim);
			sort_shell<double>(ii, ev);

			for (int r = 0; r < nDim; ++r) {
				eigenvalues[r] = ev[ii[nDim-1-r]];
				for (int c = 0; c < nDim; ++c) {
					eigenvectors[r][c] = evc[r][ii[nDim - 1 - c]];
				}
			}
		}

		//设定正负号  
		for (int i = 0; i < nDim; i++) {
			double dSumVec = 0;
			for (int j = 0; j < nDim; j++)
				dSumVec += pdblVects[j * nDim + i];
			if (dSumVec < 0) {
				for (int j = 0; j < nDim; j++)
					pdblVects[j * nDim + i] *= -1;
			}
		}

		return 1;
	}

    /*
       |A22| = 0
       x^2 + y^2 = 1
    */
    static bool solve_homo_equation2(Matrix<double> A22, double& x, double& y)
    {
        A22.rowSimplest();
        int rk = A22.rank();
        if (rk < 1) {
            return false;
        }
        if (std::abs(A22[0][0]) <= ZERO_F) {
            y = 0;
            x = 1;
        } else {
            double t = A22[0][1] / A22[0][0];
            y = 1.0 / std::sqrt(t * t + 1.0);
            x = t * y;
        }
        return true;
    }

    template<typename T>
    bool Matrix<T>::eigen_decomposition_2(const Matrix<T>& obj, Vector2<T>& eigen_value, Matrix<T>& eigen_vector, int& eigen_count)
    {
        eigen_vector.resize(2, 2);
        eigen_vector.reset(0);
        double a11 = obj[0][0], a12 = obj[0][1], a21 = obj[1][0], a22 = obj[1][1];
        double delta = (a11 + a22) * (a11 + a22) - 4.0 * (a11 * a22 - a12 * a21);
        if (std::abs(delta) <= ZERO_D) {
            double e = (a11 + a22) / 2.0;
            double data[4] = { a11 - e, a12, a21, a22 - e };
            Matrix<double> A(2, 2, data);
            double x, y;
            bool tag = solve_homo_equation2(A, x, y);
            if (!tag) {
                return false;
            }
            eigen_vector[0][0] = x;
            eigen_vector[1][0] = y;
            eigen_count = 1;
            return true;
        } else if (delta < ZERO_D) {
            return false;
        } else {
            double sqrt_delta = std::sqrt(delta);
            double e1 = ((a11 + a22) - sqrt_delta) / 2.0;
            double e2 = ((a11 + a22) + sqrt_delta) / 2.0;

            if (e1 < e2) {
                std::swap(e1, e2);
            }

            eigen_value[0] = e1;
            eigen_value[1] = e2;

            double data1[4] = { a11 - e1, a12, a21, a22 - e1 };
            Matrix<double> A(2, 2, data1);
            double x, y;
            solve_homo_equation2(A, x, y);
            eigen_vector[0][0] = x;
            eigen_vector[1][0] = y;

            double data2[4] = { a11 - e2, a12, a21, a22 - e2 };
            A = Matrix<double>(2, 2, data2);
            solve_homo_equation2(A, x, y);
            eigen_vector[0][1] = x;
            eigen_vector[1][1] = y;

            eigen_count = 2;

            return true;
        }
    }
}
