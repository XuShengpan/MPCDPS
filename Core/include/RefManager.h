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

#ifndef  MPCDPS_REFMANAGER_H
#define  MPCDPS_REFMANAGER_H

#include <cstddef>
#include <cstring>
#include <map>
#include <cassert>
#include <mutex>
#include "PublicInfo.h"
#include "MPCDPSCoreLib.h"

namespace mpcdps {

class RefManagerCore;
class MPCDPS_CORE_ITEM RefManager
{
    friend class RefManagerCore;

protected:
    RefManager();

public:
    ~RefManager();

    static RefManager* getInstance();

	void referenceAdd(void* data);

    template <typename T>
    inline void referenceReduce1(T* data)
    {
        if (data == NULL) {
            return;
        }
        _mutex.lock();
        void* ptr = static_cast<void*>(data);
        RefMap::iterator iter = _ref_map.find(ptr);
        assert(iter != _ref_map.end());
        int num = iter->second;
        assert(num > 0);
        _ref_map[ptr] = --num;
        if (num == 0) {
            _ref_map.erase(iter);
            _mutex.unlock();
            delete data;
        } else {
            _mutex.unlock();
        }
    }

    /* Reduce reference for the pointer data.
     * data is the pointer for a smart array.
     */
    template <typename T>
    inline void referenceReduce2(T* data)
    {
        if (data == NULL) {
            return;
        }
        _mutex.lock();
        void* ptr = static_cast<void*>(data);
        RefMap::iterator iter = _ref_map.find(ptr);
        assert(iter != _ref_map.end());
        int num = iter->second;
        assert(num > 0);
        _ref_map[ptr] = --num;
        if (num == 0) {
            _ref_map.erase(iter);
            _mutex.unlock();
            delete[] data;
        } else {
            _mutex.unlock();
        }
    }

private:
    typedef std::map<void*, int> RefMap;
    std::mutex  _mutex;
    RefMap  _ref_map;
};

}

#endif
