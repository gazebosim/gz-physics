/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <list>
#include <stack>
#include <unordered_map>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>

#include "AABBTree.hh"

namespace ignition {
namespace physics {
namespace tpelib {

class AABBNode
{
  public: bool IsLeaf() const;
  public: uint64_t id = 0u;
  public: std::shared_ptr<AABBNode> parent;
  public: std::shared_ptr<AABBNode> left;
  public: std::shared_ptr<AABBNode> right;
  public: math::AxisAlignedBox aabb;

  public: static uint64_t kNodeIdCounter;
};

/// \brief Private data class for AABBTree
class AABBTreePrivate
{
  public: std::shared_ptr<AABBNode> CreateNode(
      const math::AxisAlignedBox &_aabb);
  public: std::shared_ptr<AABBNode> InsertNode(
      const std::shared_ptr<AABBNode> &_parent,
      const std::shared_ptr<AABBNode> &_node);
  public: bool DetachNode(const std::shared_ptr<AABBNode> &_node);
  public: bool RemoveNode(const std::shared_ptr<AABBNode> &_node);
  public: bool UpdateNode(const std::shared_ptr<AABBNode> &_node,
      const math::AxisAlignedBox &_aabb);

  public: void CollisionPairs(
    const std::shared_ptr<AABBNode> &_left,
    const std::shared_ptr<AABBNode> &_right,
    std::vector<std::pair<uint64_t, uint64_t>> &_pairs) const;

  public: std::vector<uint64_t> Collisions(
      const std::shared_ptr<AABBNode> &_node) const;

  public: std::shared_ptr<AABBNode> root;

  public: std::unordered_map<uint64_t, std::shared_ptr<AABBNode>> nodes;
};

}
}
}

using namespace ignition;
using namespace physics;
using namespace tpelib;

uint64_t AABBNode::kNodeIdCounter = 0u;

//////////////////////////////////////////////////
bool AABBNode::IsLeaf() const
{
  return !(this->left || this->right);
}

//////////////////////////////////////////////////
AABBTree::AABBTree()
  : dataPtr(new ::tpelib::AABBTreePrivate)
{
}

//////////////////////////////////////////////////
AABBTree::~AABBTree()
{
  this->dataPtr->root.reset();
  this->dataPtr->nodes.clear();
}

//////////////////////////////////////////////////
uint64_t AABBTree::AddNode(const math::AxisAlignedBox &_aabb)
{
  std::shared_ptr<AABBNode> node = this->dataPtr->CreateNode(_aabb);

  // emtpy tree, set node to root node
  if (!this->dataPtr->root)
  {
    this->dataPtr->root = node;
  }
  else
  {
    std::shared_ptr<AABBNode> inserted =
        this->dataPtr->InsertNode(this->dataPtr->root, node);

    // if insertion was successful at root, the new inserted branch node
    // becomes root
    if (inserted)
      this->dataPtr->root = inserted;
  }

  return node->id;
}

//////////////////////////////////////////////////
bool AABBTree::RemoveNode(uint64_t _id)
{
  auto it = this->dataPtr->nodes.find(_id);
  if (it == this->dataPtr->nodes.end())
  {
    ignerr << "Unable to remove node '" << _id << "'. "
           << "Node not found." << std::endl;
    return false;
  }

  return this->dataPtr->RemoveNode(it->second);
}

//////////////////////////////////////////////////
bool AABBTree::UpdateNode(uint64_t _id, const math::AxisAlignedBox &_aabb)
{
  auto it = this->dataPtr->nodes.find(_id);
  if (it == this->dataPtr->nodes.end())
  {
    ignerr << "Unable to update node '" << _id << "'. "
           << "Node not found." << std::endl;
    return false;
  }

  return this->dataPtr->UpdateNode(it->second, _aabb);
}


//////////////////////////////////////////////////
unsigned int AABBTree::NodeCount() const
{
  return this->dataPtr->nodes.size();
}

//////////////////////////////////////////////////
void AABBTree::CollisionPairs(
    std::vector<std::pair<uint64_t, uint64_t>> &_pairs) const
{
  if (!this->dataPtr->root || this->dataPtr->root->IsLeaf())
    return;

  this->dataPtr->CollisionPairs(this->dataPtr->root->left,
      this->dataPtr->root->right, _pairs);
}

//////////////////////////////////////////////////
std::vector<uint64_t> AABBTree::Collisions(uint64_t _id) const
{
  std::vector<uint64_t> empty;
  if (!this->dataPtr->root || this->dataPtr->root->IsLeaf())
    return empty;

  auto it = this->dataPtr->nodes.find(_id);
  if (it == this->dataPtr->nodes.end())
    return empty;

  return this->dataPtr->Collisions(it->second);
}

double sa(const math::AxisAlignedBox &_aabb)
{
  auto s = _aabb.Size();
  return 2.0 * (s.X() * s.Y() + s.X() * s.Z() + s.Y() * s.Z());
}

//////////////////////////////////////////////////
std::shared_ptr<AABBNode> AABBTreePrivate::CreateNode(
const math::AxisAlignedBox &_aabb)
{
  std::shared_ptr<AABBNode> node = std::make_shared<AABBNode>();
  node->id = AABBNode::kNodeIdCounter++;
  node->aabb = _aabb;
  this->nodes[node->id] = node;
  return node;
}

/*
//////////////////////////////////////////////////
std::shared_ptr<AABBNode> AABBTreePrivate::InsertNode(
  const std::shared_ptr<AABBNode> &_parent,
  const std::shared_ptr<AABBNode> &_node)
{
  // Parent is leaf node. Create a new branch node
  // and attach _parent and _node to this new branch node
  bool addNode = false;
  if (_parent->IsLeaf())
  {
    addNode = true;
  }
  else
  {
//    auto combinedAabb = _parent->aabb;
//    combinedAabb.Merge(_node->aabb);
//    double newParentCost = 2.0 * sa(combinedAabb);
//    double pushDownCost = 2.0 * (sa(combinedAabb) - sa(_parent->aabb));
//
//    double costLeft = 0.0;
//    double costRight = 0.0;
//    if (_parent->left->IsLeaf())
//    {
//      auto aabb = _node->aabb;
//      aabb.Merge(_parent->left->aabb);
//      costLeft = sa(aabb) + pushDownCost;
//    }
//    else
//    {
//      auto aabb = _node->aabb;
//      aabb.Merge(_parent->left->aabb);
//      costLeft = (sa(aabb) - sa(_parent->left->aabb)) + pushDownCost;
//    }
//
//    if (_parent->right->IsLeaf())
//    {
//      auto aabb = _node->aabb;
//      aabb.Merge(_parent->right->aabb);
//      costRight= sa(aabb) + pushDownCost;
//    }
//    else
//    {
//      auto aabb = _node->aabb;
//      aabb.Merge(_parent->right->aabb);
//      costRight = (sa(aabb) - sa(_parent->right->aabb)) + pushDownCost;
//    }

    auto combinedAabb = _parent->aabb;
    combinedAabb.Merge(_node->aabb);
    double newParentCost = 2.0 * combinedAabb.Volume();
    double pushDownCost = 2.0 * (combinedAabb.Volume() - _parent->aabb.Volume());

    double costLeft = 0.0;
    double costRight = 0.0;
    if (_parent->left->IsLeaf())
    {
      auto aabb = _node->aabb;
      aabb.Merge(_parent->left->aabb);
      costLeft = aabb.Volume() + pushDownCost;
    }
    else
    {
      auto aabb = _node->aabb;
      aabb.Merge(_parent->left->aabb);
      costLeft = (aabb.Volume() - _parent->left->aabb.Volume()) + pushDownCost;
    }

    if (_parent->right->IsLeaf())
    {
      auto aabb = _node->aabb;
      aabb.Merge(_parent->right->aabb);
      costRight= aabb.Volume() + pushDownCost;
    }
    else
    {
      auto aabb = _node->aabb;
      aabb.Merge(_parent->right->aabb);
      costRight = (aabb.Volume() - _parent->right->aabb.Volume()) + pushDownCost;
    }

    if (newParentCost < costLeft && newParentCost < costRight)
    {
      addNode = true;
    }
    else
    {
      if (costLeft < costRight)
      {
        std::shared_ptr<AABBNode> inserted =
            this->InsertNode(_parent->left, _node);
        if (inserted)
          _parent->left = inserted;

        _parent->aabb.Merge(_parent->left->aabb);
      }
      else
      {
        std::shared_ptr<AABBNode> inserted =
            this->InsertNode(_parent->right, _node);
        if (inserted)
          _parent->right = inserted;

        _parent->aabb.Merge(_parent->right->aabb);
      }
    }
  }

  if (addNode)
  {
    math::AxisAlignedBox newAabb = _parent->aabb;
    newAabb.Merge(_node->aabb);

    std::shared_ptr<AABBNode> branch = this->CreateNode(newAabb);
    branch->parent = _parent->parent;
    _parent->parent = branch;
    _node->parent = branch;
    branch->left = _parent;
    branch->right = _node;
    return branch;
  }

  return std::shared_ptr<AABBNode>();
}
*/

//////////////////////////////////////////////////
std::shared_ptr<AABBNode> AABBTreePrivate::InsertNode(
  const std::shared_ptr<AABBNode> &_parent,
  const std::shared_ptr<AABBNode> &_node)
{
  // Parent is leaf node. Create a new branch node
  // and attach _parent and _node to this new branch node
  if (_parent->IsLeaf())
  {
    math::AxisAlignedBox newAabb = _parent->aabb;
    newAabb.Merge(_node->aabb);

    std::shared_ptr<AABBNode> branch = this->CreateNode(newAabb);
    branch->parent = _parent->parent;
    _parent->parent = branch;
    _node->parent = branch;
    branch->left = _parent;
    branch->right = _node;
    return branch;
  }
  else
  {
    // compute volume and use it as cost for adding new node
    auto leftAabb = _parent->left->aabb;
    leftAabb.Merge(_node->aabb);
    double leftVolume = leftAabb.Volume();

    auto rightAabb = _parent->right->aabb;
    rightAabb.Merge(_node->aabb);
    double rightVolume = rightAabb.Volume();

    if (leftVolume < rightVolume)
    {
      std::shared_ptr<AABBNode> inserted =
          this->InsertNode(_parent->left, _node);
      if (inserted)
        _parent->left = inserted;

      _parent->aabb.Merge(_parent->left->aabb);
    }
    else
    {
      std::shared_ptr<AABBNode> inserted =
          this->InsertNode(_parent->right, _node);
      if (inserted)
        _parent->right = inserted;

      _parent->aabb.Merge(_parent->right->aabb);
    }
  }

  return std::shared_ptr<AABBNode>();
}

//////////////////////////////////////////////////
bool AABBTreePrivate::UpdateNode(const std::shared_ptr<AABBNode> &_node,
    const math::AxisAlignedBox &_aabb)
{
  if (_node == this->root)
  {
    if (this->root->IsLeaf())
    {
      this->root->aabb = _aabb;
      return true;
    }
    else
    {
      ignerr << "Unable to update root node '" << _node->id << "'. "
             << "Node is not a leaf node" << std::endl;
      return false;
    }
  }

  // update by detach node, and reinserting it back to the tree
  bool result = this->DetachNode(_node);
  if (result)
  {
    _node->aabb = _aabb;
    std::shared_ptr<AABBNode> inserted =
        this->InsertNode(this->root, _node);

    // if insertion was successful at root, the new inserted branch node
    // becomes root
    if (inserted)
      this->root = inserted;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool AABBTreePrivate::RemoveNode(const std::shared_ptr<AABBNode> &_node)
{
  // remove root node
  if (_node == this->root)
  {
    if (this->root->IsLeaf())
    {
      this->nodes.clear();
      this->root.reset();
      return true;
    }
    else
    {
      ignerr << "Unable to remove root node '" << _node->id << "'. "
             << "Node is not a leaf node" << std::endl;
      return false;
    }
  }

  // remote other leaf nodes in the tree
  // detach and erase
  bool result = this->DetachNode(_node);
  if (result)
    this->nodes.erase(_node->id);
  return result;
}


//////////////////////////////////////////////////
bool AABBTreePrivate::DetachNode(const std::shared_ptr<AABBNode> &_node)
{
  if (!_node->IsLeaf())
  {
    ignerr << "Unable to detach node '" << _node->id << "'. "
           << "Node is not a leaf node" << std::endl;
    return false;
  }

  // if there is no parent, the node to detach is root
  auto parent = _node->parent;
  if (!parent)
  {
    ignerr << "Unable to detach root node '" << _node->id << "'. "
           << std::endl;
    return false;
  }

  // Detach target node and remove parent branch node,
  // Connect its sibling to the parent branch node's parent,
  // i.e. grandparent, if any

  auto grandparent = parent->parent;

  // find the sibiling
  std::shared_ptr<AABBNode> sibling;
  if (parent->left == _node)
    sibling = parent->right;
  else
    sibling = parent->left;

  // connect sibling to grandparent
  sibling->parent = grandparent;

  if (grandparent)
  {
    // update grandparent's child node pointers
    if (grandparent->left == parent)
      grandparent->left = sibling;
    else
      grandparent->right = sibling;

    // update AABBs up the tree
    auto nodeToUpdate = grandparent;
    while (nodeToUpdate)
    {
      nodeToUpdate->aabb = nodeToUpdate->left->aabb;
      nodeToUpdate->aabb.Merge(nodeToUpdate->right->aabb);

      nodeToUpdate = nodeToUpdate->parent;
    }
  }
  else
  {
    // if there is no grand parent, the parent should be root
    // so make the sibling the new root
    this->root = sibling;
  }

  // remove parent branch node
  this->nodes.erase(parent->id);

  // clear node pointers
  _node->parent.reset();

  return true;
}

//////////////////////////////////////////////////
void AABBTreePrivate::CollisionPairs(
    const std::shared_ptr<AABBNode> &_left,
    const std::shared_ptr<AABBNode> &_right,
    std::vector<std::pair<uint64_t, uint64_t>> &_pairs) const
{

  if (_left->IsLeaf() && _right->IsLeaf())
  {
    if (_left->aabb.Intersects(_right->aabb))
    {
      _pairs.push_back(std::make_pair(_left->id, _right->id));
    }
  }

  if (!_left->IsLeaf())
  {
    this->CollisionPairs(_left->left, _left->right, _pairs);
    if (_right->IsLeaf())
    {
      this->CollisionPairs(_right, _left->left, _pairs);
      this->CollisionPairs(_right, _left->right, _pairs);
    }
    else
    {
      this->CollisionPairs(_right->left, _left->left, _pairs);
      this->CollisionPairs(_right->left, _left->right, _pairs);
      this->CollisionPairs(_right->right, _left->left, _pairs);
      this->CollisionPairs(_right->right, _left->right, _pairs);
    }
  }

  if (!_right->IsLeaf())
  {
    this->CollisionPairs(_right->left, _right->right, _pairs);
    if (_left->IsLeaf())
    {
      this->CollisionPairs(_left, _right->left, _pairs);
      this->CollisionPairs(_left, _right->right, _pairs);
    }
  }

/*  if (_left->IsLeaf())
  {
    if (_right->IsLeaf())
    {
      std::cerr << _left->id << ":" << _right->id << " " <<
                   _left->aabb << " vs " << _right->aabb <<
                   "  % " << _left->aabb.Intersects(_right->aabb)
                   << std::endl;

      if (_left->aabb.Intersects(_right->aabb))
      {
        _pairs.push_back(std::make_pair(_left->id, _right->id));
      }
    }
    else
    {
      this->CollisionPairs(_left, _right->left, _pairs);
      this->CollisionPairs(_left, _right->right, _pairs);
      this->CollisionPairs(_right->_left, _right->right, _pairs);
    }
  }
  else
  {
    this->CollisionPairs(_left->_left, _left->right, _pairs);

    if (_right->IsLeaf())
    {
      this->CollisionPairs(_right, _left->left, _pairs);
      this->CollisionPairs(_right, _left->right, _pairs);
    }
    else
    {
      this->CollisionPairs(_right->left, _left->left, _pairs);
      this->CollisionPairs(_right->left, _left->right, _pairs);
      this->CollisionPairs(_right->right, _left->left, _pairs);
      this->CollisionPairs(_right->right, _left->right, _pairs);
    }
  }
*/
}

//////////////////////////////////////////////////
std::vector<uint64_t> AABBTreePrivate::Collisions(
    const std::shared_ptr<AABBNode> &_node) const
{
  IGN_PROFILE("AABBTree::Collisions");

  std::vector<uint64_t> collisions;

  std::stack<std::shared_ptr<AABBNode>> aabbNodes;
  aabbNodes.push(this->root->left);
  aabbNodes.push(this->root->right);
  while (!aabbNodes.empty())
  {
    std::shared_ptr<AABBNode> n = aabbNodes.top();
    aabbNodes.pop();

    if (_node == n)
      continue;

    if (_node->aabb.Intersects(n->aabb))
    {
      if (n->IsLeaf())
      {
        collisions.push_back(n->id);
      }
      else
      {
        aabbNodes.push(n->left);
        aabbNodes.push(n->right);
      }
    }
  }
  return collisions;
}

//////////////////////////////////////////////////
std::string AABBTree::DotGraphStr() const
{
  std::stringstream vertices;
  std::stringstream edges;

  if (this->dataPtr->root)
  {
    std::list<std::shared_ptr<AABBNode>> nodes;
    nodes.push_back(this->dataPtr->root);
    while (!nodes.empty())
    {
      std::shared_ptr<AABBNode> node = nodes.front();
      nodes.pop_front();

      vertices << node->id << " " << "[label=\"" << node->id << "\"];\n";

      if (node->left)
      {
        edges << node->id << " -- " << node->left->id << ";\n";
        nodes.push_back(node->left);
      }
      if (node->right)
      {
        edges << node->id << " -- " << node->right->id << ";\n";
        nodes.push_back(node->right);
      }
    }
  }

  std::string out;
  out += "graph {\n";

  out += vertices.str();
  out += edges.str();

  out += "}";
  return out;
}
