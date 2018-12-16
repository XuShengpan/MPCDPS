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

	template <typename T>
	Quaternion<T>::Quaternion() : Vector4<T>()
	{
	}

	template <typename T>
	Quaternion<T>::Quaternion(T qx, T qy, T qz, T qw) : Vector4<T>(qx, qy, qz, qw)
	{
	}

	template <typename T>
	Quaternion<T>::Quaternion(T theta_x, T theta_y, T theta_z)
	{
		Quaternion<T> qx = Quaternion<T>(std::sin(theta_x*0.5), 0.0, 0.0, std::cos(theta_x*0.5));
		Quaternion<T> qy = Quaternion<T>(0.0, std::sin(theta_y*0.5), 0.0, std::cos(theta_y*0.5));
		Quaternion<T> qz = Quaternion<T>(0.0, 0.0, std::sin(theta_z*0.5), std::cos(theta_z*0.5));
		*this = qz + qy + qx; // qx + qy + qz;
	}

	template <typename T>
	void Quaternion<T>::setNovatelEulerAngle(T roll, T pitch, T heading)
	{
		Quaternion<T> qz = Quaternion<T>(0.0, 0.0, -heading);
		Quaternion<T> qx = Quaternion<T>(pitch, 0.0, 0.0);
		Quaternion<T> qy = Quaternion<T>(0.0, roll, 0.0);
		*this = qz + qx + qy;
	}

	template <typename T>
	Quaternion<T>::Quaternion(const Vector3<T>& axis, double theta)
	{
		if (std::abs(theta) > ZERO_D) {
			double s = std::sin(theta*0.5);
			this->_data[0] = axis[0] * s;
			this->_data[1] = axis[1] * s;
			this->_data[2] = axis[2] * s;
			this->_data[3] = std::cos(theta*0.5);
		} else {
			this->_data[0] = 0;
			this->_data[1] = 0;
			this->_data[2] = 0;
			this->_data[3] = 1;
		}
	}

	//construct a quaternion with a rotation matrix
	template <typename T>
	Quaternion<T>::Quaternion(const Matrix<T>& mat)
	{
		T* rot = mat.buffer();
		T* quat = this->buffer();
		T d0 = rot[0 * 3 + 0];
		T d1 = rot[1 * 3 + 1];
		T d2 = rot[2 * 3 + 2];
		T xx = 1.0 + d0 - d1 - d2;      // from the diagonal of the rotation
		T yy = 1.0 - d0 + d1 - d2;      // matrix, find the terms in
		T zz = 1.0 - d0 - d1 + d2;      // each Quaternion component
		T rr = 1.0 + d0 + d1 + d2;      // (using the fact that rr + xx + yy + zz=4)

		double max = rr;         // find the maximum of all terms;
		if (xx > max) {
			max = xx;      // dividing by the maximum makes
		}
		if (yy > max) {
			max = yy;      // the computations more stable
		}
		if (zz > max) {
			max = zz;      // and avoid division by zero
		}

		if (rr == max) {
			T r4 = T(std::sqrt(rr) * 2);
			quat[3] = r4 / 4.0;
			r4 = T(1) / r4;
			quat[0] = (rot[2 * 3 + 1] - rot[1 * 3 + 2]) * r4;     // find other components from
			quat[1] = (rot[0 * 3 + 2] - rot[2 * 3 + 0]) * r4;     // off diagonal terms of
			quat[2] = (rot[1 * 3 + 0] - rot[0 * 3 + 1]) * r4;     // rotation matrix.
		} else if (xx == max) {
			T x4 = T(std::sqrt(xx) * 2);
			quat[0] = x4 / 4.0;
			x4 = T(1) / x4;
			quat[1] = (rot[1 * 3 + 0] + rot[0 * 3 + 1]) * x4;
			quat[2] = (rot[2 * 3 + 0] + rot[0 * 3 + 2]) * x4;
			quat[3] = (rot[2 * 3 + 1] - rot[1 * 3 + 2]) * x4;
		} else if (yy == max) {
			T y4 = T(std::sqrt(yy) * 2);
			quat[1] = y4 / 4.0;
			y4 = T(1) / y4;
			quat[0] = (rot[1 * 3 + 0] + rot[0 * 3 + 1]) * y4;
			quat[2] = (rot[2 * 3 + 1] + rot[1 * 3 + 2]) * y4;
			quat[3] = (rot[0 * 3 + 2] - rot[2 * 3 + 0]) * y4;
		} else {
			T z4 = T(std::sqrt(zz) * 2);
			quat[2] = z4 / 4.0;
			z4 = T(1) / z4;
			quat[0] = (rot[2 * 3 + 0] + rot[0 * 3 + 2]) * z4;
			quat[1] = (rot[2 * 3 + 1] + rot[1 * 3 + 2]) * z4;
			quat[3] = (rot[1 * 3 + 0] - rot[0 * 3 + 1]) * z4;
		}
	}

	template <typename T>
	Quaternion<T>::Quaternion(const Quaternion<T>& q) :Vector4<T>(q)
	{
	}

	template <typename T>
	Quaternion<T>& Quaternion<T>::operator = (const Quaternion<T>& q)
	{
		for (int i = 0; i < 4; ++i) {
			this->_data[i] = q[i];
		}
		return *this;
	}

	template <typename T>
	Quaternion<T> Quaternion<T>::operator + (const Quaternion<T>& q) const
	{
		const T r1 = this->_data[3];
		const T r2 = q[3];

		Vector3d i1(this->buffer()), i2(q.buffer());
		T real_v = r1 * r2 - i1.dot(i2);
		Vector3d img = i1.cross(i2);
		for (uint i = 0; i < 3; ++i) {
			img[i] += (i2[i] * r1) + (i1[i] * r2);
		}

		return Quaternion<T>(img[0], img[1], img[2], real_v);
	}

	// Return a rotation with the same rotation direction, but scalor * the rotation angle
	template <typename T>
	Quaternion<T> Quaternion<T>::operator * (const T scalor) const
	{
        Vector3d rv;
        double angle;
        getAxisAngle(rv, angle);
        return Quaternion(rv, scalor * angle);
	}

	template <typename T>
	Quaternion<T> Quaternion<T>::inverseQuaternion() const
	{
		return Quaternion<T>(-this->_data[0], -this->_data[1], -this->_data[2], this->_data[3]);
	}

	template <typename T>
	Matrix<T> Quaternion<T>::getRotationMatrix() const
	{
		const T* quat = this->buffer();
		T rot[9];
		T x2 = quat[0] * quat[0];
		T xy = quat[0] * quat[1];
		T rx = quat[3] * quat[0];
		T y2 = quat[1] * quat[1];
		T yz = quat[1] * quat[2];
		T ry = quat[3] * quat[1];
		T z2 = quat[2] * quat[2];
		T zx = quat[2] * quat[0];
		T rz = quat[3] * quat[2];
		T r2 = quat[3] * quat[3];
		rot[0] = r2 + x2 - y2 - z2;     // fill diagonal terms
		rot[4] = r2 - x2 + y2 - z2;
		rot[8] = r2 - x2 - y2 + z2;
		rot[3] = 2 * (xy + rz);     // fill off diagonal terms
		rot[6] = 2 * (zx - ry);
		rot[7] = 2 * (yz + rx);
		rot[1] = 2 * (xy - rz);
		rot[2] = 2 * (zx + ry);
		rot[5] = 2 * (yz - rx);
		return Matrix<T>(3, 3, rot);
	}

	template <typename T>
    void Quaternion<T>::getAxisAngle(Vector3d& axis, double& theta) const
	{
        double m = Vector3d(this->_data).norm();
        if (m > ZERO_D) {
            theta = std::atan2(m, this->_data[3]) * 2.0;
            if (theta > M_PI) {
                theta -= M_PI * 2.0;
            }
            axis[0] = this->_data[0] / m;
            axis[1] = this->_data[1] / m;
            axis[2] = this->_data[2] / m;
            axis.normalize();
        } else {
            axis[0] = 0;
            axis[1] = 0;
            axis[2] = 0;
        }
	}

	//The Euler angles are define as yaw-pitch-roll.
	template <typename T>
	Vector3<T> Quaternion<T>::getEulerAngles() const
	{
		const T* quat = this->buffer();
		Vector3<T> euler;
		double q0 = quat[3], q1 = quat[0], q2 = quat[1], q3 = quat[2];
		euler[0] = std::atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2));   // roll, theta_x
		euler[1] = std::asin(2 * (q0*q2 - q3*q1));         // pitch, theta_y
		euler[2] = std::atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3));   // yaw, theta_z
		return euler;
	}

	//The Euler angles are define as yaw-pitch-roll.
	template <typename T>
	Vector3<T> Quaternion<T>::getNovatelEulerAngles() const
	{
		Vector3<T> euler_vector;
		Matrix<T> rot = getRotationMatrix();
		euler_vector[0] = -std::atan2(-rot[0][1], rot[1][1]);   // yaw
		euler_vector[1] = std::asin(rot[2][1]);       // pitch
		euler_vector[2] = std::atan2(-rot[2][0], rot[2][2]);    // roll
		return euler_vector;
	}

	//Rotate a 3D point.
	template <typename T>
	Vector3<T> Quaternion<T>::getRotatePoint(const Vector3<T>& point) const
	{
		T real = this->_data[3];
		Vector3<T> qv(this->buffer());
		Vector3<T> i_x_v = qv.cross(point);
		Vector3<T> i_x_v_2 = i_x_v.cross(qv);
		return point + i_x_v * T(2 * real) - i_x_v_2*T(2);
	}

	// Get an interpolation for a constant motion between this and another quaternion.
	template <typename T>
	Quaternion<T> Quaternion<T>::slerp(const T& t, const Quaternion<T>& rhs) const
	{
		const T* quat = this->buffer();
		T one = static_cast<T>(1);
		VectorK<T, 4> q1(this->buffer());
		VectorK<T, 4> q2(rhs.buffer());
		T q1q2 = q1.dot(q2);
		T q1_norm_inverse = one / q1.norm();
		T q2_norm_inverse = one / q2.norm();
		T cos_theta = q1q2 * q1_norm_inverse * q2_norm_inverse;
		T slerp_quat[4];
		T th = static_cast<T>(1.0 - 1e-6);
		if (cos_theta < 0) {
			cos_theta = -cos_theta;
			q1_norm_inverse = -q1_norm_inverse;
		}
		if (cos_theta < th) {
			T theta = acos(cos_theta);
			T sin_theta_inverse = one / sin(theta);
			T w1 = sin((one - t) * theta) * sin_theta_inverse * q1_norm_inverse;
			T w2 = sin(t * theta) * sin_theta_inverse * q2_norm_inverse;
			slerp_quat[0] = quat[0] * w1 + rhs[0] * w2;
			slerp_quat[1] = quat[1] * w1 + rhs[1] * w2;
			slerp_quat[2] = quat[2] * w1 + rhs[2] * w2;
			slerp_quat[3] = quat[3] * w1 + rhs[3] * w2;
		} else {
			T w1 = (one - t) * q1_norm_inverse;
			T w2 = t * q2_norm_inverse;
			slerp_quat[0] = quat[0] * w1 + rhs[0] * w2;
			slerp_quat[1] = quat[1] * w1 + rhs[1] * w2;
			slerp_quat[2] = quat[2] * w1 + rhs[2] * w2;
			slerp_quat[3] = quat[3] * w1 + rhs[3] * w2;
		}
		return Quaternion<T>(slerp_quat[0], slerp_quat[1], slerp_quat[2], slerp_quat[3]);
	}