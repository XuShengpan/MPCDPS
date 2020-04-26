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

template <typename T>
SmartPointer<T>::SmartPointer(T* data): mData(data)
{
	if(mData)  RefManager::getInstance()->referenceAdd(mData);
}

template <typename T>
SmartPointer<T>::SmartPointer(const SmartPointer<T>& otherObj)
:mData(otherObj.mData)
{
	if(mData)  RefManager::getInstance()->referenceAdd(mData);
}

template <typename T>
SmartPointer<T>::~SmartPointer(void)
{
	clear();
}

template <typename T>
SmartPointer<T>::operator T* () const
{
	return mData;
}

template <typename T>
T& SmartPointer<T>::operator* () const
{
	return *mData;
}

template <typename T>
T* SmartPointer<T>::operator-> () const
{
	return mData;
}

template <typename T>
T* SmartPointer<T>::operator= (T* data)
{
	if(data == mData)  return mData;
	if(mData)  clear();
	mData = data;
	if(mData) RefManager::getInstance()->referenceAdd(mData);
	return mData;
}

template <typename T>
SmartPointer<T>& SmartPointer<T>::operator= (const SmartPointer<T>& otherObj)
{
	if(otherObj.mData == mData)  return *this;
	if(mData)  clear();
	mData = otherObj.mData;
	if(mData) RefManager::getInstance()->referenceAdd(mData);
	return *this;

}

template <typename T>
bool SmartPointer<T>::operator== (T* data) const
{
	return mData == data;
}

template <typename T>
bool SmartPointer<T>::operator!= (T* data) const
{
	return mData != data;
}

template <typename T>
void SmartPointer<T>::clear()
{
    if (mData) {
        RefManager::getInstance()->referenceReduce1<T>(mData);
    }
	mData = NULL;
}

template <typename T>
T* SmartPointer<T>::get() const
{
	return mData;
};




