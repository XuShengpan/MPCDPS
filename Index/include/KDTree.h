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

#ifndef  MPCDPS_KDTREE_H
#define MPCDPS_KDTREE_H

#include <vector>
#include <stack>
#include <cmath>
#include <algorithm>
#include "SmartArray2D.h"
#include "FixedSizeMap.h"
#include "PublicFunc.h"

namespace mpcdps {

    class KDTreeNode;
    class KDTreeLeafNode;
    class KDTreeBranchNode;

    /*class KDTree.
     *  T: data type
     *  K: search dimension
     *  K1: data dimension, K1 >= K
     */
    template <typename  T, int K, int K1 = K>
    class KDTree
    {
    public:
        /*Default constructor.*/
        KDTree();

        /*Destructor.*/
        ~KDTree();

        /*Type define for key type.*/
        typedef T KeyType;

        /*Type define for element type.*/
        typedef T*  ElemType;

        /*Layer count of the tree.*/
        int   layerCount()  const;

        /*Root node of the tree.*/
        KDTreeNode* rootNode()  const;

        /*Test if the tree is empty.*/
        bool empty() const;

        /*Clear the tree.*/
        void clear();

        /*Set elements.*/
        void setElements(const SmartArray2D<T, K1>& elems);

		const T* getElement(int i) const;

        /*Build the KDTree.
         *  minvalue[i] is minimum value for i'th dimension.
         */
        void build(const std::vector<int>& elem_ids, const T minvalue[], const T maxvalue[], const T nodesize[]);

        /*Search the nearest elements with radius, return element id.
         * If no element found, return -1.
         * dist2 is square distance between the two elements.
         */
        int searchNearest(const T* elem, double radius, double& dist2) const;

        /*Search k nearest elements with radius, return element ids.
        * If no element found, return std::vector<int>().
        * dist2_list is square distance list.
        */
        std::vector<int> searchKNearest(
            const T* elem, int k, double radius,
            std::vector<double>& dist2_list)  const;

        /*Search elements within circle, return element ids.
        * If no element found, return std::vector<int>().
        * dist2_list is square distance list.
        */
        std::vector<int> searchRadius(
            const T* elem, double radius,
            std::vector<double>& dist2_list) const;

    protected:
        KDTreeLeafNode* createLeafNode(
            int layer_id, const std::vector<int>& elem_ids);

        double  squareDistance(
			const T* elem1, const T* elem2) const;

    protected:
        SmartArray2D<T, K1> _elems;  /*tree elements. */
        int _layerCount;                             /*tree layer count. */
        KDTreeBranchNode*  _root;       /*tree root. */

        T  _dx[K];   /*tree size. */
    };

#include "KDTree.inl"

}


#endif