/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
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

#ifndef GZ_PHYSICS_DETAIL_ENTITY_STORAGE_HH
#define GZ_PHYSICS_DETAIL_ENTITY_STORAGE_HH

#include <cstddef>
#include <optional>
#include <unordered_map>
#include <vector>

namespace gz
{
namespace physics
{
namespace detail
{
// INTERNAL IMPLEMENTATION. DO NOT USE OUTSIDE OF gz-physics

/// \brief A class used to store mappings between entity ids and objects that
/// represent the entity in the physics engine
/// \tparam Value1 This is typically the ``*Info` struct that contains the
/// underlying pointer to the physics object as well as other information such
/// as local name of the entity.
/// \tparam Key2 A secondary key that can be used to lookup the entity. For
/// example, this could be a std::string so that we can do a lookup based on the
/// name of the entity.
///
/// \note This class can be used to store entities across multiple worlds as
/// long as the IDs are unique. As such, the IndexMap type is a map from the
/// parent ID to a vector of object IDs. This is used to determine the index of
/// an entity within the container. However, if EntityStorage objects are
/// hierarchically stored in `*Info` structs, the indexInContainerToID will only
/// have one entry.
template <typename Value1, typename Key2 = Value1>
struct EntityStorage
{
  /// \brief Map from an entity ID to its corresponding object
  std::unordered_map<std::size_t, Value1> idToObject;

  /// \brief Map from an object pointer (or other unique key) to its entity ID
  std::unordered_map<Key2, std::size_t> objectToID;

  using IndexMap = std::unordered_map<std::size_t, std::vector<std::size_t>>;
  /// \brief The key represents the parent ID. The value represents a vector of
  /// the objects' IDs. The key of the vector is the object's index within its
  /// container. This is used by World and Model objects, which don't know their
  /// own indices within their containers as well as Links, whose indices might
  /// change when constructing joints.
  ///
  /// The container type for World is Engine.
  /// The container type for Model is World.
  /// The container type for Link is Model.
  ///
  /// Joints are contained in Models, but they know their own indices within
  /// their Models, so we do not need to use this field for Joints
  IndexMap indexInContainerToID;

  /// \brief Map from an entity ID to its index within its container
  std::unordered_map<std::size_t, std::size_t> idToIndexInContainer;

  /// \brief Map from an entity ID to the ID of its container
  std::unordered_map<std::size_t, std::size_t> idToContainerID;

  Value1 &operator[](const std::size_t _id)
  {
    return idToObject[_id];
  }

  Value1 &at(const std::size_t _id)
  {
    return idToObject.at(_id);
  }

  const Value1 &at(const std::size_t _id) const
  {
    return idToObject.at(_id);
  }

  std::optional<Value1> MaybeAt(const std::size_t _id) const
  {
    auto it = this->idToObject.find(_id);
    if (it != this->idToObject.end())
    {
      return it->second;
    }
    return std::nullopt;
  }

  Value1 &at(const Key2 &_key)
  {
    return idToObject.at(objectToID.at(_key));
  }

  const Value1 &at(const Key2 &_key) const
  {
    return idToObject.at(objectToID.at(_key));
  }

  std::size_t size() const
  {
    return idToObject.size();
  }

  std::size_t IdentityOf(const Key2 &_key) const
  {
    return objectToID.at(_key);
  }

  bool HasEntity(const Key2 &_key) const
  {
    return objectToID.find(_key) != objectToID.end();
  }

  bool HasEntity(const std::size_t _id) const
  {
    return idToObject.find(_id) != idToObject.end();
  }

  void AddEntity(std::size_t _id, const Value1 &_value1, const Key2 &_key,
                 std::size_t _containerID)
  {
    this->idToObject[_id] = _value1;
    this->objectToID[_key] = _id;
    std::vector<std::size_t> &indexInContainerToIDVector =
        this->indexInContainerToID[_containerID];
    const std::size_t indexInContainer = indexInContainerToIDVector.size();

    this->idToIndexInContainer[_id] = indexInContainer;
    indexInContainerToIDVector.push_back(_id);
    this->idToContainerID[_id] = _containerID;
  }

  bool RemoveEntity(const Key2 &_key)
  {
    auto entIter = this->objectToID.find(_key);
    if (entIter!= this->objectToID.end())
    {
      std::size_t entId = entIter->second;

      // Check if we are keeping track of the index of this entity in its
      // container
      auto contIter = this->idToContainerID.find(entId);
      if (contIter != this->idToContainerID.end())
      {
        std::size_t contId = contIter->second;
        std::size_t entIndex = this->idToIndexInContainer.at(entId);

        // house keeping
        // The key in indexInContainerToID is the index of the vector so erasing
        // the element automatically decrements the index of the rest of the
        // elements of the vector. The indices in idToIndexInContainer, however,
        // are stored as numbers (as values in the map). We need to decrement
        // all the indices greater than the index of the model we are removing.
        for (auto indIter =
                 this->indexInContainerToID[contId].begin() + entIndex + 1;
             indIter != this->indexInContainerToID[contId].end(); ++indIter)
        {
          // decrement the index (the value of the map)
          --this->idToIndexInContainer[*indIter];
        }

        this->idToIndexInContainer.erase(entId);
        this->indexInContainerToID[contId].erase(
            this->indexInContainerToID[contId].begin() + entIndex);
        this->idToContainerID.erase(entId);
      }

      this->objectToID.erase(entIter);
      this->idToObject.erase(entId);
      return true;
    }
    return false;
  }
};

}
}

}
#endif

