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

#ifndef  MPCDPS_POINT3_H
#define MPCDPS_POINT3_H

#include <vector>
#include "Tuple.h"
#include "VectorK.h"

namespace mpcdps {

/*Point of 3-dimension. */
template <typename T>
class  Point3: public Tuple<T, 3>
{
public:
    /* Default constructor.*/
    Point3()
    {
    }

    /*Construct a Point3 with (x,y,z).*/
    Point3(T x, T y, T z) 
    {
        this->_data[0] = x;
        this->_data[1] = y;
        this->_data[2] = z;
    }

    /*Construct a Point3 as the given data buffer.*/
    Point3(const T* data):Tuple<T, 3>(data)
    {
    }

    /*Copy constructor.*/
    Point3(const Point3<T>& obj):Tuple<T, 3>(obj)
    {
    }

    /*Add a Vector3 to get a new point.*/
    Point3<T> operator + (const Vector3<T>& vect) const
    {
        Point3<T> point;
        point[0] = this->_data[0] + vect[0];
        point[1] = this->_data[1] + vect[1];
        point[2] = this->_data[2] + vect[2];
        return point;
    }

    /*Subtract a Vector3 to get a new point.*/
    Point3<T> operator - (const Vector3<T>& vect) const
    {
        Point3<T> point;
        point[0] = this->_data[0] - vect[0];
        point[1] = this->_data[1] - vect[1];
        point[2] = this->_data[2] - vect[2];
        return point;
    }

    /*Subtract a point to get a Vector3.*/
    Vector3<T> operator - (const Point3<T>& point) const
    {
        Vector3<T> vect;
        vect[0] = this->_data[0] - point[0];
        vect[1] = this->_data[1] - point[1];
        vect[2] = this->_data[2] - point[2];
        return vect;
    }

    /*Add a Vector3 and update this point.*/
    void operator += (const Vector3<T>& vect)
    {
        this->_data[0] += vect[0];
        this->_data[1] += vect[1];
        this->_data[2] += vect[2];
    }

    /*Subtract a Vector3 and update this point.*/
    void operator -= (const Vector3<T>& vect)
    {
        this->_data[0] -= vect[0];
        this->_data[1] -= vect[1];
        this->_data[2] -= vect[2];
    }

    void setX(T x) { this->_data[0] = x;} 
    void setY(T y) { this->_data[1] = y;} 
    void setZ(T z) { this->_data[2] = z;} 

    T x() const { return this->_data[0];}
    T y() const { return this->_data[1];}
    T z() const { return this->_data[2];}
};

typedef Point3 <double> Point3d;
typedef Point3 <float>    Point3f;

typedef std::vector <Point3d> Point3dList;

}

#endif
