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

#ifndef MPCDPS_PUBLICINFO_H
#define MPCDPS_PUBLICINFO_H

/* This file defines some macros that will be used frequently during mathematical operations. */

#include <cfloat>
#include <cmath>
#include <cassert>

#ifndef  NO_USE_MPCDPS_UTYPE
#define USE_MPCDPS_UTYPE
 typedef unsigned char uchar;
 typedef unsigned short ushort;
 typedef unsigned int uint;
#endif

const double ZERO_F = 1e-06;
const double ZERO_D = 1e-08;

#define Square(x) ((x)*(x))

#define  MAXV(a, b) ((a)>(b)?(a):(b))
#define  MINV(a, b) ((a) < (b)?(a):(b))

#define  Deg2Rad(t) ((t)*M_PI/180.0)
#define  Rad2Deg(t) ((t)/M_PI*180.0)

#define  RANDOM_INT(n_max)  (rand()/double(RAND_MAX) * double(n_max))

template <typename T>
void SWAP(T& a, T& b)
{
	T t = a;
	a = b;
	b = t;
}

#define  LOCATE(v,v0,dv) ((v)-(v0))/(dv)

#define  SKIP(v0,dv,t)   ((v0) + (t)*(dv))

#define  PointDistant2_2D(v0,v1) (Square(v1[0]-v0[0]) + Square(v1[1]-v0[1])) 
#define  PointDistant2_3D(v0,v1) (Square(v1[0]-v0[0]) + Square(v1[1]-v0[1]) + Square(v1[2]-v0[2]))

#endif
