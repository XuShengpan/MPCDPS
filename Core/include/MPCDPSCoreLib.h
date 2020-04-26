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

#ifndef MPCDPSLID_H
#define MPCDPSLID_H

#if defined(_WINDOWS)
	#if defined(MPCDPS_EXPORT)
		// For the DLL library.
		#define MPCDPS_CORE_ITEM __declspec(dllexport)
   #else
		// For a client of the DLL library.
		#define MPCDPS_CORE_ITEM   __declspec(dllimport)
	#endif
#else
    //For Apple and Linux
	#define MPCDPS_CORE_ITEM
#endif

#endif