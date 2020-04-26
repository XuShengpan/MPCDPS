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

#ifndef MPCDPS_RTREE2_H
#define MPCDPS_RTREE2_H

#include <vector>
#include "Rect2.h"
#include "RTreeTemplate.h"

namespace mpcdps {
    /* \brief RTree for 2D.
     * T: type of rect, e.g. Rect2f; DataType: type of data that saved in a rect node.
    */
	template<typename T, typename DataType>
	class RTree2
	{
	public:
		RTree2();
		~RTree2();

		struct RTreeElem
		{
			DataType data;
			Rect2<T> rect;

			RTreeElem(DataType d, const Rect2<T>& r):data(d), rect(r)
			{
			}
		};

		/*
		   assert:
		   none of one element is contained by another.
		*/
		void addElem(const RTreeElem& elem);

		/*
		  All of the elements that intersect with rect will be added to the result set.
		*/
		std::vector<DataType> searchElems(const Rect2<T>& rect);

		/*
		  All of the elements that contains the searching point.
		*/
		std::vector<DataType> searchElems(const Point2<T>& point);

		void removeElem(const RTreeElem& elem);

		int elemCount();

		bool isEmpty();

		void clear();

		Rect2<T> getRootRect();

	protected:
	   typedef RTree<DataType, T, 2, float> RTCore;
	   RTCore mTree;
		
	};

#include "RTree2.inl"

}


#endif