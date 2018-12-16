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

#ifndef  MPCDPS_QUATERNION_H
#define MPCDPS_QUATERNION_H

#include "Matrix.h"
#include "VectorK.h"
#include "PublicInfo.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

    /* Class Quaternion.*/
    template <typename T>
    class Quaternion: public Vector4<T>
    {
    public:
        /* Default constructor.*/
        Quaternion();

        /* Constructor: construct a quaternion with (qx, qy, qz, qw).*/
        Quaternion(T qx, T qy, T qz, T qw);

        /*Construct a quaternion from Euler angles.
         *  The Euler angles are defines as a rotation about the Z axis (yaw), followed by Y (pitch), 
         * followed by X (roll) finally, using a fixed reference frame.
         */
        Quaternion(T theta_x, T theta_y, T theta_z);
        
        /*Construct a quaternion from a rotation axis and a rotation angle. */
        Quaternion(const Vector3<T>& axis, double angle_rad);

        /*Construct a quaternion with a rotation matrix. */
        Quaternion(const Matrix<T>& R);

        /*Copy constructor. */
        Quaternion(const Quaternion<T>& rth);

		/*Set the quaternion with the Euler angles followed novatel standard.
		*
		* The Euler angles are defines as a rotation about the Y axis, followed by X,
		* followed by Z finally, using a fixed reference frame.
		*/
		void setNovatelEulerAngle(T roll, T pitch, T heading);

        /*operator = */
        Quaternion<T>& operator = (const Quaternion<T>& q);

        /*operator + */
        Quaternion<T> operator + (const Quaternion<T>& q) const;

        /*operator *
         * Return a rotation with the same rotation direction, but scalor * the rotation angle.*/
        Quaternion<T> operator * (const T scalor) const;

        /*Return inverse quaternion. */
        Quaternion<T> inverseQuaternion() const;

        /*Return rotation matrix. */
        Matrix<T> getRotationMatrix() const;

        /*
            Get (axis, angle) where axis is a unit vector and theta is in rad.
        */
        void getAxisAngle(Vector3d& axis, double& theta) const;

        /*Return rotation vector. 
         * The Euler angles are defined as yaw-pitch-roll.*/
        Vector3<T> getEulerAngles() const;

        /*Return novatel rotation vector.
        * The Euler angles are defined as yaw-pitch-roll.*/
        Vector3<T> getNovatelEulerAngles() const;

        /*Rotate a 3D point.*/
        Vector3<T> getRotatePoint(const Vector3<T>& point) const;

        /*Get an interpolation for a constant motion between this and another quaternion.*/
        Quaternion<T> slerp(const T& t, const Quaternion<T>& rhs) const;
    };

#include "Quaternion.inl"

}

#endif