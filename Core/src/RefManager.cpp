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

#include "RefManager.h"

namespace mpcdps {

    class RefManagerCore
    {
    public:
        RefManagerCore()
        {
        }

        ~RefManagerCore()
        {
        }

        RefManager* get()
        {
            return &_mger;
        }

    protected:
        RefManager _mger;
    };

    static RefManagerCore _core;

    RefManager::RefManager()
    {

    }

    RefManager::~RefManager()
    {

    }

    RefManager* RefManager::getInstance()
    {
        return _core.get();
    }

    void RefManager::referenceAdd(void* data)
    {
        if (data) {
            _mutex.lock();
            int num = _ref_map[data];
            _ref_map[data] = num + 1;
            _mutex.unlock();
        }
    }
}

