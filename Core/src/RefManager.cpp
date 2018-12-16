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

