/*
**********************************************************************
*
* This file is a part of library MPCDPS(Massive Point Cloud Data Processing System).
* It is a free program and it is protected by the license GPL-v3.0, you may not use the
* file except in compliance with the License.
*
* Copyright(c) 2013 - 2019 Xu Shengpan, all rights reserved.
*
* Email: jack_1227x@163.com
*
**********************************************************************
*/

/*
bool MySearchCallback(int id, void* arg) 
{
printf("Hit data rect %d\n", id);
return true; // keep going
}
*/

template<typename T, typename DataType>
RTree2<T, DataType>::RTree2()
{
}

template<typename T, typename DataType>
RTree2<T, DataType>::~RTree2()
{

}

template<typename T, typename DataType>
void RTree2<T, DataType>::addElem(const RTreeElem& elem)
{
	mTree.Insert(elem.rect.min_ptr(), elem.rect.max_ptr(), elem.data);
}

template<typename T, typename DataType>
std::vector<DataType> RTree2<T, DataType>::searchElems(const Rect2<T>& rect)
{
	std::vector<DataType> ids;
	if(isEmpty())
		return ids;
	mTree.Search(rect.min_ptr(), rect.max_ptr(), ids);
	return ids;
}

template<typename T, typename DataType>
std::vector<DataType> RTree2<T, DataType>::searchElems(const Point2<T>& point)
{
	std::vector<DataType> ids;
	if(isEmpty())
		return ids;
	mTree.Search(point, ids);
	return ids;
}

template<typename T, typename DataType>
int RTree2<T, DataType>::elemCount()
{
	return mTree.Count();
}

template<typename T, typename DataType>
void RTree2<T, DataType>::removeElem(const RTreeElem& elem)
{
	mTree.Remove(elem.rect.min_ptr(), elem.rect.max_ptr(), elem.data);
}

template<typename T, typename DataType>
bool RTree2<T, DataType>::isEmpty()
{
	return mTree.Count() == 0;
}

template<typename T, typename DataType>
void RTree2<T, DataType>::clear()
{
	mTree.RemoveAll();
}

template<typename T, typename DataType>
Rect2<T> RTree2<T, DataType>::getRootRect()
{
	typename RTCore::Rect r = mTree.NodeCover(mTree.GetRootNode());
	return Rect2<T>(r.m_min, r.m_max);
}