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

#include "Transform.h"
#include "Quaternion.h"

namespace mpcdps {

    Transform Transform::operator * (const Transform& oth) const
    {
        Transform tr;
        tr._R = _R + oth._R;
        tr._R.normalize();
        tr._t = _R.getRotatePoint(oth._t) + _t;
        return tr;
    }

    Matrix<double> Transform::mat() const
    {
        mpcdps::Matrix<double> r = _R.getRotationMatrix();
        double data[] = {
            r[0][0], r[0][1], r[0][2], _t[0],
            r[1][0], r[1][1], r[1][2], _t[1],
            r[2][0], r[2][1], r[2][2], _t[2],
            0, 0, 0, 1
        };
        return Matrix<double>(4, 4, data);
    }

    Transform Transform::inverse() const
    {
        Transform tr;
        tr._R = _R.inverseQuaternion();
        tr._t = -1.0 * tr._R.getRotationMatrix() * _t;
        return tr;
    }

    Vector3d Transform::getTransformPoint(const Vector3d& pt) const
    {
        return _R.getRotationMatrix() * pt + _t;
    }

	Matrix<double> getTransformMatrix(const double* translation, const double* rotation)
	{
		/*
		Matrix<double> T = Matrix<double>::identityMatrix(4);
		Quaternion<double> q(rotation[0], rotation[1], rotation[2], rotation[3]);
		T.setBlock(q.getRotationMatrix(), 0, 0, 3, 3);
		T[0][3] = translation[0];
		T[1][3] = translation[1];
		T[2][3] = translation[2];
		return T; */

		Matrix<double> Rot = Quaternion<double>(rotation[0], rotation[1], rotation[2], rotation[3]).getRotationMatrix();
		double t[] = {
			Rot[0][0], Rot[0][1], Rot[0][2], translation[0], 
			Rot[1][0], Rot[1][1], Rot[1][2], translation[1],
			Rot[2][0], Rot[2][1], Rot[2][2], translation[2],
			0, 0, 0, 1
		};
		return Matrix<double>(4, 4, t);
	}

    //translation[3], rotation[4]
    Matrix<double> getInverseTransformMatrix(const double* translation, const double* rotation)
    {
        Quaternion<double> q(rotation[0], rotation[1], rotation[2], rotation[3]);
        Quaternion<double> q1 = q.inverseQuaternion();
        Matrix<double> R1 = q1.getRotationMatrix();
        Vector3d t1 = -1 * R1*Vector3d(translation);
        double data[] = {
            R1[0][0], R1[0][1], R1[0][2], t1[0],
            R1[1][0], R1[1][1], R1[1][2], t1[1],
            R1[2][0], R1[2][1], R1[2][2], t1[2],
            0, 0, 0, 1
        };
        return Matrix<double>(4, 4, data);
    }

	Matrix<double> getTranslateMatrix(const Vector3d& vect)
	{
		/*
		Matrix<double> T = Matrix<double>::identityMatrix(4);
		T[0][3] = vect[0];
		T[1][3] = vect[1];
		T[2][3] = vect[2]; */

		double t[] = {
			1, 0, 0, vect[0],
			0, 1, 0, vect[1],
			0, 0, 1, vect[2],
			0, 0, 0, 1,
		};

		return Matrix<double>(4, 4, t);
	}

	Matrix<double> getRotateMatrix(const Point3d& pos, Vector3d axis, double theta)
	{
		/*
		axis.normalize();
		Matrix<double> Rot = Quaternion<double>(axis, theta).getRotationMatrix();
		Matrix<double> R = Matrix<double>::identityMatrix(4);
		R.setBlock(Rot,0,0,3,3);
		Vector3d T(pos.buffer());
		return getTranslateMatrix(T)*R*getTranslateMatrix(-T); */

		//accelerate
		axis.normalize();
		Matrix<double> Rot = Quaternion<double>(axis, theta).getRotationMatrix();
		Vector3d RT = Rot * Vector3d(pos.buffer());

		double* rd = Rot.buffer();
		double t[] = {
			rd[0], rd[1], rd[2], -RT[0] + pos[0],
			rd[3], rd[4], rd[5], -RT[1] + pos[1],
			rd[6], rd[7], rd[8], -RT[2] + pos[2],
			0, 0, 0, 1
		};
		return Matrix<double>(4, 4, t);
	}

	Matrix<double> getScaleMatrix(const Point3d& pos, const Vector3d& scalar)
	{
		/*
		Matrix<double> R = Matrix<double>::identityMatrix(4);
		double d = scalar.norm();
		if(d < ZEAR_F)
			return R;
		R[0][0] = scalar[0];
		R[1][1] = scalar[1];
		R[2][2] = scalar[2];
		Vector3d T(pos.buffer());
		return getTranslateMatrix(T)*R*getTranslateMatrix(-T); */

		double t[] = {
			scalar[0], 0, 0, pos[0] - scalar[0] * pos[0],
			0, scalar[1], 0, pos[1] - scalar[1] * pos[1],
			0, 0, scalar[2], pos[2] - scalar[2] * pos[2],
			0, 0, 0, 1
		};
		return Matrix<double>(4, 4, t);
	}

}