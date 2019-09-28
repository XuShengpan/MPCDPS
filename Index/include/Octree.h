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

#ifndef  MPCDPS_OCTREE_H
#define MPCDPS_OCTREE_H

#include <vector>
#include <stack>
#include <Box3.h>
#include <SmartArray2D.h>
#include <SmartPointer.h>

namespace mpcdps {

struct OctreeNode
{
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

    int index;  //data index
};

template<typename T>
class Octree
{
protected:
    SmartArray2D<T, 3> _points;
    Point3d _center;
    int _min_points_per_nodes;
    float _max_leaf_node_size;

    std::vector<std::vector<int> > _leaf_data;
    SmartPointer<OctreeBranchNode> _root;

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

    void set_min_points_per_node(int n)
    {
        _min_points_per_nodes = n;
    }

    void set_max_leaf_node_size(float sz)
    {
        _max_leaf_node_size = sz;
    }

    void build(const SmartArray2D<T, 3>& points, 
        const std::vector<int>& ptids, Box3d box)
    {
        _center = box.center();
        _points = points;
        double xmin = box.min(0) - _center[0];
        double xmax = box.max(0) - _center[0];
        double ymin = box.min(1) - _center[1];
        double ymax = box.max(1) - _center[1];
        double zmin = box.min(2) - _center[2];
        double zmax = box.max(2) - _center[2];
        
        _root = new OctreeBranchNode;
        _root->box = Box3f(xmin, xmax, ymin, ymax, zmin, zmax);
        _leaf_data.clear();

        std::stack<StackNode> stk;
        stk.push(StackNode(_root, ptids));

        while (!stk.empty()) {
            StackNode node = stk.top();
            stk.pop();
            node_split(node, stk);
        }
    }

    std::vector<std::vector<int> >& get_leaves()
    {
        return _leaf_data;
    }

    const std::vector<std::vector<int> >& get_leaves() const
    {
        return _leaf_data;
    }

    const OctreeBranchNode* getRoot() const
    {
        return _root;
    }

protected:
    int get_index(float v, float v0)
    {
        return (v > v0) ? 1 : 0;
    }

    void node_split(StackNode& node, std::stack<StackNode>& stk)
    {
        const Box3f& box = node.branch_node->box;
        Point3f cnt = box.center();
        double x0 = cnt[0] + _center[0];
        double y0 = cnt[1] + _center[1];
        double z0 = cnt[2] + _center[2];

        std::vector<std::vector<int> > ptids_children(8);
        const std::vector<int>& ptids = node.ptids;
        
        const T* vtx = NULL;
        for (int i = 0; i < ptids.size(); ++i) {
            vtx = _points[ptids[i]];
            int k = get_index(vtx[2], z0) * 4 + get_index(vtx[1], y0) * 2 + get_index(vtx[0], x0);
            ptids_children[k].push_back(ptids[i]);
        }

        float dx = box.length(0) * 0.5;
        float dy = box.length(1) * 0.5;
        float dz = box.length(2) * 0.5;

        bool child_need_split = (dx > _max_leaf_node_size) && (dy > _max_leaf_node_size) && (dz > _max_leaf_node_size);

        for (int i = 0; i < 8; ++i) {
            if (ptids_children[i].empty()) {
                continue;
            }

            int h = i / 4;
            int r = (i - h * 4) / 2;
            int c = i % 2;

            Box3f box_i = Box3f(
                box.min(0) + c * dx,
                box.min(0) + (c + 1) * dx,
                box.min(1) + r * dy,
                box.min(1) + (r + 1) * dy,
                box.min(2) + h * dz,
                box.min(2) + (h + 1) * dz);

            if (child_need_split && ptids_children[i].size() > _min_points_per_nodes) {
                OctreeBranchNode* child_i = new OctreeBranchNode;
                child_i->box = box_i;
                node.branch_node->children[i] = child_i;
                stk.push(StackNode(child_i, ptids_children[i]));
            } else {
                OctreeLeafNode* child_i = new OctreeLeafNode;
                child_i->box = box_i;
                _leaf_data.push_back(ptids_children[i]);
                child_i->index = _leaf_data.size() - 1;
                node.branch_node->children[i] = child_i;
            }
        }
    }
};

}

#endif