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

#ifndef  MPCDPS_OCTREE_H
#define MPCDPS_OCTREE_H

#include <vector>
#include <stack>
#include "Box3.h"
#include "SmartArray2D.h"
#include "SmartPointer.h"
#include "BoxGetter.h"
#include "PublicFunc.h"

namespace mpcdps {

class OctreeNode
{
public:
    OctreeNode() {}
    virtual ~OctreeNode() {}

    Box3f box;
    virtual bool isLeaf() const = 0;
};

class OctreeBranchNode: public OctreeNode
{
public:
    OctreeBranchNode()
    {
        for (int i = 0; i < 8; ++i) {
            children[i] = NULL;
        }
    }

    ~OctreeBranchNode()
    {
        for (int i = 0; i < 8; ++i) {
            if (children[i]) {
                delete children[i];
                children[i] = NULL;
            }
        }
    }

    virtual bool isLeaf() const
    {
        return false;
    }

    OctreeNode* children[8];
};

class OctreeLeafNode : public OctreeNode
{
public:
    OctreeLeafNode(){}
    ~OctreeLeafNode(){}

    virtual bool isLeaf() const
    {
        return true;
    }

    std::vector<int> ptids;
};

template<typename T>
class Octree
{
protected:
    SmartArray2D<T, 3> _points;
    int _min_points_per_node;
    T _max_leaf_size[3];

    SmartPointer<OctreeNode> _root;

protected:
    struct StackNode
    {
        StackNode() :branch_node(NULL) {}
        StackNode(OctreeBranchNode* bn, const std::vector<int>& ptids_node)
            : branch_node(bn), ptids(ptids_node)
        {
        }
        StackNode(const StackNode& rth)
            :branch_node(rth.branch_node), ptids(rth.ptids)
        {

        }

        OctreeBranchNode* branch_node;
        std::vector<int> ptids;
    };

public:
    Octree()
    {
        _root = NULL;
    }

    ~Octree()
    {
    }

    void setMinPointsForNode(int n)
    {
        _min_points_per_node = n;
    }

    void setMaxLeafShape(T max_shape[3])
    {
        _max_leaf_size[0] = max_shape[0];
        _max_leaf_size[1] = max_shape[1];
        _max_leaf_size[2] = max_shape[2];
    }

    void build(const SmartArray2D<T, 3>& points, std::vector<int> ptids = std::vector<int>())
    {
        if (ptids.empty()) {
            ptids = make_vector<int>(points.size());
        }
        _points = points;

        auto box = getBox(ptids);

        if (!isSplit(box, ptids.size())) {
            OctreeLeafNode* node = new OctreeLeafNode;
            node->box = box;
            node->ptids = ptids;
            _root = node;
            return;
        }

        OctreeBranchNode* branch = new OctreeBranchNode;
        branch->box = box;
        _root = branch;

        std::stack<StackNode> stk;
        stk.push(StackNode(branch, ptids));

        while (!stk.empty()) {
            StackNode node = stk.top();
            stk.pop();
            node_split(node, stk);
        }
    }

    const OctreeNode* getRoot() const
    {
        return _root;
    }

    const T* getVertex(int ptid) const
    {
        return _points[ptid];
    }

protected:

    inline int getIndex(T v, T v0) const
    {
        return (v < v0) ? 0 : 1;
    }

    inline int getIndex(T* pt, float cnt[3]) const
    {
        int ix = getIndex(pt[0], cnt[0]);
        int iy = getIndex(pt[1], cnt[1]);
        int iz = getIndex(pt[2], cnt[2]);
        return (iz * 4 + iy * 2 + ix);
    }

    mpcdps::Box3f getBox(const std::vector<int>& ptids) const
    {
        BoxGetter bg;
        T* vtx = NULL;
        for (auto ptid: ptids) {
            vtx = _points[ptid];
            bg.add_point(vtx[0], vtx[1], vtx[2]);
        }

        return bg.getBox<float>();
    }

    bool isSplit(const Box3f& box, int points_size) const
    {
        if ((box.length(0) > _max_leaf_size[0] ||
            box.length(1) > _max_leaf_size[1] ||
            box.length(2) > _max_leaf_size[2]) && points_size > _min_points_per_node)
            return true;
        else
            return false;
    }

    void node_split(StackNode& node, std::stack<StackNode>& stk) const
    {
        std::vector<std::vector<int> > ptids_children(8);
        int k = 0;

        Point3f cnt = node.branch_node->box.center();
        float* cnt_ptr = cnt.buffer();
        for (auto ptid : node.ptids) {
            k = getIndex(_points[ptid], cnt_ptr);
            ptids_children[k].push_back(ptid);
        }

        for (int i = 0; i < 8; ++i) {
            auto box = getBox(ptids_children[i]);
            if (isSplit(box, ptids_children[i].size())) {
                OctreeBranchNode* branch = new OctreeBranchNode;
                branch->box = box;
                stk.push(StackNode(branch, ptids_children[i]));
                node.branch_node->children[i] = branch;
            } else if(!ptids_children[i].empty()){
                OctreeLeafNode* leaf = new OctreeLeafNode;
                leaf->box = box;
                leaf->ptids = ptids_children[i];
                node.branch_node->children[i] = leaf;
            }
        }
    }
};

}

#endif