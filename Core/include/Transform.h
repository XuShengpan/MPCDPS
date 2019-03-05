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

#ifndef  MPCDPS_TRANSFORM_H
#define MPCDPS_TRANSFORM_H

#include "Matrix.h"
#include "VectorK.h"
#include "Point3.h"
#include "Quaternion.h"

namespace mpcdps {

    class MPCDPS_CORE_ITEM Transform
    {
    protected:
        Quaternion<double> _R;
        Vector3d _t;

    public:
        Transform()
        {
            _R = Quaternion<double>(0, 0, 0, 1);
            _t = Vector3d(0, 0, 0);
        }

        Transform(const Vector3d& t, const Quaternion<double>& R)
        {
            _R = R;
            _t = t;
        }

        /* t:  x, y, z
            R: qx, qy, qz, qw
        */
        Transform(const double* t, const double* R)
        {
            _t[0] = t[0];
            _t[1] = t[1];
            _t[2] = t[2];

            _R[0] = R[0];
            _R[1] = R[1];
            _R[2] = R[2];
            _R[3] = R[3];
        }

        /* t:  x, y, z
            R: qx, qy, qz, qw
        */
        void set(const double* t, const double* R)
        {
            _t[0] = t[0];
            _t[1] = t[1];
            _t[2] = t[2];

            _R[0] = R[0];
            _R[1] = R[1];
            _R[2] = R[2];
            _R[3] = R[3];
        }

        /* t:  x, y, z
            R: qx, qy, qz, qw
        */
        void get(double t[3], double R[4]) const
        {
            t[0] = _t[0];
            t[1] = _t[1];
            t[2] = _t[2];

            R[0] = _R[0];
            R[1] = _R[1];
            R[2] = _R[2];
            R[3] = _R[2];
        }
        
        void setTranslation(const Vector3d& t)
        {
            _t = t;
        }

        void setRotation(const Quaternion<double>& R)
        {
            _R = R;
        }

        Quaternion<double> rotation() const
        {
            return _R;
        }

        Vector3d translation() const
        {
            return _t;
        }

        double* getRotationBuffer()
        {
            return _R.buffer();
        }

        double* getTranslationBuffer()
        {
            return _t.buffer();
        }

        Transform operator * (const Transform& oth) const;
        Matrix<double> mat() const;
        Transform inverse() const;

        /*
           for point (3, 1) form X,
           X1 = RX + t;
           return X1;
        */
        Vector3d getTransformPoint(const Vector3d& pt) const;
    };

	//translation[3], rotation[4]; return a 4x4 matrix.
    MPCDPS_CORE_ITEM Matrix<double> getTransformMatrix(const double* translation, const double* rotation);

    //translation[3], rotation[4]; return a 4x4 matrix.
    MPCDPS_CORE_ITEM Matrix<double> getInverseTransformMatrix(const double* translation, const double* rotation);

	//return a 4x4 matrix.
    MPCDPS_CORE_ITEM Matrix<double> getTranslateMatrix(const Vector3d& vect);

	//return a 4x4 matrix.
    MPCDPS_CORE_ITEM Matrix<double> getRotateMatrix(const Point3d& pos, Vector3d axis, double theta);

	//scale around pos; return a 4x4 matrix.
    MPCDPS_CORE_ITEM Matrix<double> getScaleMatrix(const Point3d& pos, const Vector3d& scalar);

}

#endif