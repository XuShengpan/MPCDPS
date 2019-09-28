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

class KDTreeNode
{
protected:
	KDTreeNode(int layer) :_layer(layer), _empty(true)
	{
	}

public:
	virtual ~KDTreeNode()
	{
	}

	virtual std::vector<int> elemList() const = 0;
	virtual bool isLeafNode() const = 0;
	bool empty() const { return _empty; }
	int   layer() const { return _layer; }

protected:
	int    _layer;
	bool  _empty;
};

class KDTreeLeafNode :public KDTreeNode
{
public:
	KDTreeLeafNode(int layer_id, const std::vector<int>& elem_ids)
		:KDTreeNode(layer_id), _elem_ids(elem_ids)
	{
		_empty = false;
	}

	std::vector<int> elemList() const { return _elem_ids; }
	bool isLeafNode() const { return true; }

private:
	std::vector<int> _elem_ids;
};

class KDTreeBranchNode : public KDTreeNode
{
public:
	KDTreeBranchNode(int layer_id)
		:KDTreeNode(layer_id), _leftChild(NULL), _rightChild(NULL), _dim(-1)
	{
	}

	~KDTreeBranchNode()
	{
		if (_leftChild) {
			delete _leftChild;
		}
		_leftChild = NULL;
		if (_rightChild) {
			delete _rightChild;
		}
		_rightChild = NULL;
	}

	std::vector<int> elemList() const
	{
		if (empty()) {
			return std::vector<int>();
		}

		std::vector<int> elems_left;
		std::vector<int> elems_right;
		if (_leftChild) {
			elems_left = _leftChild->elemList();
		}
		if (_rightChild) {
			elems_right = _rightChild->elemList();
		}
		elems_left.insert(elems_left.end(), elems_right.begin(), elems_right.end());
		return elems_left;
	}

	bool isLeafNode() const
	{
		return false;
	}

	void setLeftChild(KDTreeNode* lchild)
	{
		_leftChild = lchild;
		if (_empty && _leftChild) {
			if (!_leftChild->empty()) {
				_empty = false;
			}
		}
	}

	void setRightChild(KDTreeNode* rchild)
	{
		_rightChild = rchild;
		if (_empty && _rightChild) {
			if (!_rightChild->empty()) {
				_empty = false;
			}
		}
	}

	KDTreeNode* leftChild()
	{
		return _leftChild;
	}

	KDTreeNode* rightChild()
	{
		return _rightChild;
	}

	double  _key;
	int       _dim;   //start with 0, belongs to 0~k-1.

protected:
	KDTreeNode* _leftChild;
	KDTreeNode* _rightChild;
};

//-------------------------------------------------------------------------------------//

template <typename T, int K>
struct StackNode
{
	KDTreeBranchNode* branch;
	T minv[K];
	T maxv[K];
	std::vector<int> ids;

	StackNode() :branch(NULL)
	{
	}

	StackNode(KDTreeBranchNode* node, const T* vmin, const T* vmax,
		const std::vector<int>& elem_ids)
	{
		branch = node;
		for (int i = 0; i < K; ++i) {
			minv[i] = vmin[i];
			maxv[i] = vmax[i];
		}
		ids = elem_ids;
	}

	StackNode(const StackNode& rth)
	{
		branch = rth.branch;
		for (int i = 0; i < K; ++i) {
			minv[i] = rth.minv[i];
			maxv[i] = rth.maxv[i];
		}
		ids = rth.ids;
	}
};

//-------------------------------------------------------------------------------------//

template <typename  T, int K, int K1>
KDTree<T, K, K1>::KDTree() :_layerCount(0), _root(NULL)
{
}

template <typename  T, int K, int K1>
KDTree<T, K, K1>::~KDTree()
{
	clear();
}

template <typename  T, int K, int K1>
int   KDTree<T, K, K1>::layerCount()  const
{
	return _layerCount;
}

template <typename  T, int K, int K1>
KDTreeNode* KDTree<T, K, K1>::rootNode()  const
{
	return _root;
}

template <typename  T, int K, int K1>
bool KDTree<T, K, K1>::empty() const
{
	return _layerCount == 0;
}

template <typename  T, int K, int K1>
void KDTree<T, K, K1>::clear()
{
	if (_root) {
		delete _root;
		_root = 0;
	}

	_elems.clear();
	_layerCount = 0;
}

template <typename  T, int K, int K1>
KDTreeLeafNode* KDTree<T, K, K1>::createLeafNode(
	int layer_id, const std::vector<int>& elem_ids)
{
	KDTreeLeafNode* node = new KDTreeLeafNode(layer_id, elem_ids);
	_layerCount = std::max(_layerCount, layer_id + 1);
	return node;
}

template <typename  T, int K, int K1>
void KDTree< T, K, K1>::setElements(
	const SmartArray2D<T, K1>& elemList)
{
	clear();
	_elems = elemList;
}

template <typename  T, int K, int K1>
const T* KDTree< T, K, K1>::getElement(int i) const
{
	return _elems[i];
}

template <typename  T, int K, int K1>
void KDTree<T, K, K1>::build(
    const std::vector<int>& elem_ids, 
    const T minvalue[], const T maxvalue[], const T nodesize[])
{
	if (elem_ids.empty()) {
		return;
	}
	for (int i = 0; i < K; ++i) {
		_dx[i] = nodesize[i];
	}

	KDTreeBranchNode* branch = new KDTreeBranchNode(0);
	_root = branch;

	std::stack<StackNode<T, K> > stk;
	StackNode<T, K> node(branch, minvalue, maxvalue, elem_ids);
	stk.push(node);

	{
		T left_min[K];
		T left_max[K];
		T right_min[K];
		T right_max[K];
		std::vector<int> left_elm;
		std::vector<int> right_elm;
		std::vector<int> ids;
		T split_key;
		int dim;
		while (!stk.empty()) {
			node = stk.top();
			stk.pop();
			branch = node.branch;
			ids = node.ids;

			T t1;
			T t = 0;
			dim = -1;
			for (int i = 0; i < K; ++i) {
				t1 = (node.maxv[i] - node.minv[i]) / _dx[i];
				if (t1 > t) {
					t = t1;
					dim = i;
				}
			}

			assert(t > 1);
			int n = ids.size();
			std::vector<T> values(n);
			for (int i = 0; i < n; ++i) {
				values[i] = _elems[ids[i]][dim];
			}

			sort_shell_syn(values, ids);
			n /= 2;

			split_key = values[n];
			left_elm = std::vector<int>(ids.begin(), ids.begin() + n);
			right_elm = std::vector<int>(ids.begin() + n, ids.end());
			for (int k1 = 0; k1 < K; ++k1) {
				left_min[k1] = node.minv[k1];
				left_max[k1] = node.maxv[k1];

				right_min[k1] = node.minv[k1];
				right_max[k1] = node.maxv[k1];
			}

			left_max[dim] = split_key;
			right_min[dim] = split_key;

			branch->_dim = dim;
			branch->_key = split_key;

			//left
			t = 0;
			for (int i = 0; i < K; ++i) {
				t1 = (left_max[i] - left_min[i]) / _dx[i];
				if (t1 > t) {
					t = t1;
				}
			}
			if (t < 2.0 || left_elm.size() < 200) {
				branch->setLeftChild(
					createLeafNode(branch->layer() + 1, left_elm));
			} else {
				KDTreeBranchNode* node =
					new KDTreeBranchNode(branch->layer() + 1);
				branch->setLeftChild(node);
				stk.push(StackNode<T, K>(node, left_min, left_max, left_elm));
			}

			//right
			t = 0;
			for (int i = 0; i < K; ++i) {
				t1 = (right_max[i] - right_min[i]) / _dx[i];
				if (t1 > t) {
					t = t1;
				}
			}
			if (t < 2.0 || left_elm.size() < 200) {
				branch->setRightChild(
					createLeafNode(branch->layer() + 1, right_elm));
			} else {
				KDTreeBranchNode* node =
					new KDTreeBranchNode(branch->layer() + 1);
				branch->setRightChild(node);
				stk.push(StackNode<T, K>(node, right_min, right_max, right_elm));
			}

		}
	}
}

template <typename  T, int K, int K1>
double  KDTree<T, K, K1>::squareDistance(
	const T* elem1, const T* elem2) const
{
	double d = 0, d1;
	for (int i = 0; i < K; ++i) {
		d1 = elem1[i] - elem2[i];
		d += d1*d1;
	}
	return d;
}

template <typename  T, int K, int K1>
int KDTree<T, K, K1>::searchNearest(const T* elem, double radius, double& dist2) const
{
	std::stack<KDTreeNode*> stk;
	stk.push(_root);

	int id = -1;
	dist2 = radius * radius;
	KDTreeNode* node;

	while (!stk.empty()) {
		node = stk.top();
		stk.pop();
		if (node == NULL) {
			continue;
		}
		if (node->isLeafNode()) {
			KDTreeLeafNode* leaf = dynamic_cast <KDTreeLeafNode*>(node);
			std::vector<int> ids = leaf->elemList();
			double d2;
			ElemType elemi;
			for (int i = 0; i < ids.size(); ++i) {
				elemi = _elems[ids[i]];
				if (elemi == elem) {
					continue;
				}
				d2 = squareDistance(elemi, elem);
				if (d2 <= dist2) {
					dist2 = d2;
					id = ids[i];
					if (d2 < 0.001) {
						return id;
					}
				}
			}
		} else {
			KDTreeBranchNode* branch = dynamic_cast <KDTreeBranchNode*>(node);
			int dim = branch->_dim;
			double d = branch->_key - elem[dim];
			if (d * d < dist2) {
				stk.push(branch->leftChild());
				stk.push(branch->rightChild());
			} else {
				if (d < 0) {
					stk.push(branch->rightChild());
				} else {
					stk.push(branch->leftChild());
				}
			}
		}
	}
	return id;
}

template <typename  T, int K, int K1>
std::vector<int> KDTree<T, K, K1>::searchKNearest(
	const T* elem,
	int k,
	double radius,
	std::vector<double>& dist2_list) const
{
	FixedSizeMap<double, int> queue_map(k);
	KDTreeNode* node = NULL;
	std::stack<KDTreeNode*> stk;
	stk.push(_root);
	double dist2 = radius * radius;

	while (!stk.empty()) {
		node = stk.top();
		stk.pop();
		if (node == NULL) {
			continue;
		}
		if (node->isLeafNode()) {
			KDTreeLeafNode* leaf = dynamic_cast <KDTreeLeafNode*>(node);
			std::vector<int> ids = leaf->elemList();
			double d2;
			ElemType elemi;
			for (int i = 0; i < ids.size(); ++i) {
				elemi = _elems[ids[i]];
				d2 = squareDistance(elemi, elem);
				if (d2 <= dist2) {
					queue_map.insert(d2, ids[i]);
					if (queue_map.full()) {
						dist2 = queue_map.tailKey();
					}
				}
			}
		} else {
			KDTreeBranchNode* branch = dynamic_cast <KDTreeBranchNode*>(node);
			int dim = branch->_dim;
			double d = branch->_key - elem[dim];
			if (d * d < dist2) {
				stk.push(branch->leftChild());
				stk.push(branch->rightChild());
			} else {
				if (d < 0) {
					stk.push(branch->rightChild());
				} else {
					stk.push(branch->leftChild());
				}
			}
		}
	}

	dist2_list.clear();
	std::vector<int> elem_ids;
	queue_map.elemList(dist2_list, elem_ids);
	return elem_ids;
}

template <typename  T, int K, int K1>
std::vector<int> KDTree<T, K, K1>::searchRadius(
	const T* elem,
	double radius,
	std::vector<double>& dist2_list) const
{
	dist2_list.clear();
	std::vector<int> elem_ids;
	KDTreeNode* node = NULL;
	std::stack<KDTreeNode*> stk;
	stk.push(_root);
	const double dist2 = radius * radius;

	std::vector<int> ids;

	while (!stk.empty()) {
		node = stk.top();
		stk.pop();
		if (node == NULL) {
			continue;
		}
		if (node->isLeafNode()) {
			KDTreeLeafNode* leaf = dynamic_cast <KDTreeLeafNode*>(node);
			ids = leaf->elemList();
			double d2;
			ElemType elemi;
			for (int i = 0; i < ids.size(); ++i) {
				elemi = _elems[ids[i]];
				d2 = squareDistance(elemi, elem);
				if (d2 <= dist2) {
					elem_ids.push_back(ids[i]);
					dist2_list.push_back(d2);
				}
			}
		} else {
			KDTreeBranchNode* branch = dynamic_cast <KDTreeBranchNode*>(node);
			int dim = branch->_dim;
			double d = branch->_key - elem[dim];
			if (d * d < dist2) {
				stk.push(branch->leftChild());
				stk.push(branch->rightChild());
			} else {
				if (d < 0) {
					stk.push(branch->rightChild());
				} else {
					stk.push(branch->leftChild());
				}
			}
		}
	}
	return elem_ids;
}