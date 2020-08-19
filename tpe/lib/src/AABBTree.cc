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

#include "aabb_tree/AABBTree.h"

#include "AABBTree.hh"

namespace ignition {
namespace physics {
namespace tpelib {

/// \brief Private data class for AABBTreeIface
class AABBTreeIfacePrivate
{
  /// \brief Pointer to the AABB tree
  public: std::unique_ptr<AABBTree> aabbTree;

  /// \brief A map of node id and its AABB object in the tree
  public: std::unordered_map<uint64_t, std::shared_ptr<IAABB>> nodeIds;
};

}
}
}

using namespace ignition;
using namespace physics;
using namespace tpelib;

//////////////////////////////////////////////////
AABBTreeIface::AABBTreeIface()
  : dataPtr(new ::tpelib::AABBTreeIfacePrivate)
{
  this->dataPtr->aabbTree = std::make_unique<AABBTree>(100000);
}

//////////////////////////////////////////////////
AABBTreeIface::~AABBTreeIface()
{
}

//////////////////////////////////////////////////
void AABBTreeIface::AddNode(uint64_t _id, const math::AxisAlignedBox &_aabb)
{
  std::shared_ptr<IAABB> obj = std::make_shared<IAABB>();
  obj->aabb.minX = _aabb.Min().X();
  obj->aabb.minY = _aabb.Min().Y();
  obj->aabb.minZ = _aabb.Min().Z();
  obj->aabb.maxX = _aabb.Max().X();
  obj->aabb.maxY = _aabb.Max().Y();
  obj->aabb.maxZ = _aabb.Max().Z();
  this->dataPtr->aabbTree->insertObject(obj);

  this->dataPtr->nodeIds[_id] = obj;
}

//////////////////////////////////////////////////
bool AABBTreeIface::RemoveNode(uint64_t _id)
{
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to remove node '" << _id << "'. "
           << "Node not found." << std::endl;
    return false;
  }

  this->dataPtr->aabbTree->removeObject(it->second);
  this->dataPtr->nodeIds.erase(it);
  return true;
}

//////////////////////////////////////////////////
bool AABBTreeIface::UpdateNode(uint64_t _id, const math::AxisAlignedBox &_aabb)
{
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to update node '" << _id << "'. "
           << "Node not found." << std::endl;
    return false;
  }

  it->second->aabb.minX = _aabb.Min().X();
  it->second->aabb.minY = _aabb.Min().Y();
  it->second->aabb.minZ = _aabb.Min().Z();
  it->second->aabb.maxX = _aabb.Max().X();
  it->second->aabb.maxY = _aabb.Max().Y();
  it->second->aabb.maxZ = _aabb.Max().Z();

  this->dataPtr->aabbTree->updateObject(it->second);
  return true;
}


//////////////////////////////////////////////////
unsigned int AABBTreeIface::NodeCount() const
{
  return this->dataPtr->nodeIds.size();
}

//////////////////////////////////////////////////
std::vector<uint64_t> AABBTreeIface::Collisions(uint64_t _id) const
{
  std::vector<uint64_t> result;
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to compute collisions for node '" << _id << "'. "
           << "Node not found." << std::endl;
    return result;
  }

  auto list = this->dataPtr->aabbTree->queryOverlaps(it->second);
  for (auto &objIt : list)
  {
    for (auto &nodeIt : this->dataPtr->nodeIds)
    {
      if (objIt == nodeIt.second)
      {
        result.push_back(nodeIt.first);
        break;
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////
math::AxisAlignedBox AABBTreeIface::AABB(uint64_t _id) const
{
  auto it = this->dataPtr->nodeIds.find(_id);
  if (it == this->dataPtr->nodeIds.end())
  {
    ignerr << "Unable to get AABB for node '" << _id << "'. "
           << "Node not found." << std::endl;
    return math::AxisAlignedBox();
  }
  return math::AxisAlignedBox(
      math::Vector3d(
      it->second->aabb.minX, it->second->aabb.minY, it->second->aabb.minZ),
      math::Vector3d(
      it->second->aabb.maxX, it->second->aabb.maxY, it->second->aabb.maxZ));
}

//////////////////////////////////////////////////
bool AABBTreeIface::HasNode(uint64_t _id) const
{
  auto it = this->dataPtr->nodeIds.find(_id);
  return it != this->dataPtr->nodeIds.end();
}

//////////////////////////////////////////////////
std::string AABBTreeIface::DotGraphStr() const
{
/*  std::stringstream vertices;
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
*/
  return std::string();
}
