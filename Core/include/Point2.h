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

#ifndef  MPCDPS_POINT2_H
#define MPCDPS_POINT2_H

#include <vector>
#include "Tuple.h"
#include "VectorK.h"

namespace mpcdps {

/*Point of 2-dimension.*/
template <typename T>
class  Point2 : public Tuple<T, 2>
{
public:
    /*Default constructor.*/
    Point2()
    {
    }

    /*Construct a Point2 with (x,y).*/
    Point2(T x, T y)
    {
        this->_data[0] = x;
        this->_data[1] = y;
    }

    /*Construct a Point2 as the given data buffer.*/
    Point2(const T* data):Tuple<T, 2>(data)
    {
    }

    /*Copy constructor.*/
    Point2(const Point2<T>& obj):Tuple<T, 2>(obj.buffer())
    {
    }

    /*Add a Vector2 to get a new point.*/
    Point2<T> operator + (const Vector2<T>& vect) const
    {
        Point2<T> point;
        point[0] = this->_data[0] + vect[0];
        point[1] = this->_data[1] + vect[1];
        return point;
    }

    /*Subtract a Vector2 to get a new point.*/
    Point2<T> operator - (const Vector2<T>& vect) const
    {
        Point2<T> point;
        point[0] = this->_data[0] - vect[0];
        point[1] = this->_data[1] - vect[1];
        return point;
    }

    /*Subtract a point to get a Vector2.*/
    Vector2<T> operator - (const Point2<T>& point) const
    {
        Vector2<T> vect;
        vect[0] = this->_data[0] - point[0];
        vect[1] = this->_data[1] - point[1];
        return vect;
    }

    /*Add a Vector2 and update this point.*/
    void operator += (const Vector2<T>& vect)
    {
        this->_data[0] += vect[0];
        this->_data[1] += vect[1];
    }

    /*Subtract a Vector2 and update this point.*/
    void operator -= (const Vector2<T>& vect)
    {
        this->_data[0] -= vect[0];
        this->_data[1] -= vect[1];
    }

    void setX(T x) { this->_data[0] = x;}
    void setY(T y) { this->_data[1] = y;}

    T x() const { return this->_data[0];}
    T y() const { return this->_data[1];}

};

typedef Point2<double> Point2d;
typedef Point2<float>    Point2f;

typedef std::vector<Point2d> Point2dList;

}

#endif
