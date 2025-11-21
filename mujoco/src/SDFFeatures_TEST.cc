/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <test/Utils.hh>

#include "test/common_test/Worlds.hh"

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    // physics::GetEntities,
    physics::sdf::ConstructSdfModel,
    // physics::sdf::ConstructSdfNestedModel,
    physics::sdf::ConstructSdfWorld
> { };

using World = physics::World3d<TestFeatureList>;
using WorldPtr = physics::World3dPtr<TestFeatureList>;
using ModelPtr = physics::Model3dPtr<TestFeatureList>;
using LinkPtr = physics::Link3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);

  plugin::PluginPtr mujoco =
      loader.Instantiate("gz::physics::mujoco::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(mujoco);
  return engine;
}

enum class LoaderType
{
  Whole,
  Piecemeal
};

/////////////////////////////////////////////////
WorldPtr LoadWorldWhole(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());
  for (const auto & error : errors) {
    std::cout << error << std::endl;
  }

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return world;
}

/////////////////////////////////////////////////
static gz::math::Pose3d ResolveSdfPose(
    const ::sdf::SemanticPose &_semPose)
{
  gz::math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  EXPECT_TRUE(errors.empty()) << errors;
  return pose;
}

static sdf::JointAxis ResolveJointAxis(const sdf::JointAxis &_unresolvedAxis)
{
  gz::math::Vector3d axisXyz;
  const sdf::Errors resolveAxisErrors = _unresolvedAxis.ResolveXyz(axisXyz);
  EXPECT_TRUE(resolveAxisErrors.empty()) << resolveAxisErrors;

  sdf::JointAxis resolvedAxis = _unresolvedAxis;

  const sdf::Errors setXyzErrors = resolvedAxis.SetXyz(axisXyz);
  EXPECT_TRUE(setXyzErrors.empty()) << setXyzErrors;

  resolvedAxis.SetXyzExpressedIn("");
  return resolvedAxis;
}

/////////////////////////////////////////////////
/// Downstream applications, like gz-sim, use this way of world construction
// WorldPtr LoadWorldPiecemeal(const std::string &_world)
// {
//   auto engine = LoadEngine();
//   EXPECT_NE(nullptr, engine);
//   if (nullptr == engine)
//     return nullptr;
//
//   sdf::Root root;
//   const sdf::Errors &errors = root.Load(_world);
//   EXPECT_EQ(0u, errors.size()) << errors;
//
//   EXPECT_EQ(1u, root.WorldCount());
//   const sdf::World *sdfWorld = root.WorldByIndex(0);
//   EXPECT_NE(nullptr, sdfWorld);
//   if (nullptr == sdfWorld)
//     return nullptr;
//
//   sdf::World newWorld;
//   newWorld.SetName(sdfWorld->Name());
//   newWorld.SetGravity(sdfWorld->Gravity());
//   auto world = engine->ConstructWorld(newWorld);
//   if (nullptr == world)
//     return nullptr;
//
//   std::unordered_map<const sdf::Model *, ModelPtr> modelMap;
//   std::unordered_map<const sdf::Link *, LinkPtr> linkMap;
//
//   auto createModel = [&](const sdf::Model *_model,
//                          const sdf::Model *_parentModel = nullptr) {
//     ASSERT_NE(nullptr, _model);
//     sdf::Model newSdfModel;
//     newSdfModel.SetName(_model->Name());
//     newSdfModel.SetRawPose(ResolveSdfPose(_model->SemanticPose()));
//     newSdfModel.SetStatic(_model->Static());
//     newSdfModel.SetSelfCollide(_model->SelfCollide());
//
//     ModelPtr newModel;
//     if (nullptr != _parentModel)
//     {
//       auto it = modelMap.find(_parentModel);
//       ASSERT_TRUE(it != modelMap.end());
//       newModel = it->second->ConstructNestedModel(newSdfModel);
//     }
//     else
//     {
//       newModel = world->ConstructModel(newSdfModel);
//     }
//
//     EXPECT_NE(nullptr, newModel);
//     if (nullptr != newModel)
//     {
//       modelMap[_model] = newModel;
//     }
//   };
//
//   for (uint64_t i = 0; i < sdfWorld->ModelCount(); ++i)
//   {
//     const auto *model = sdfWorld->ModelByIndex(i);
//     createModel(model);
//     for (uint64_t nestedInd = 0; nestedInd < model->ModelCount(); ++nestedInd)
//     {
//       createModel(model->ModelByIndex(nestedInd), model);
//     }
//   }
//
//   for (auto [sdfModel, physModel] : modelMap)
//   {
//     for (uint64_t li = 0; li < sdfModel->LinkCount(); ++li)
//     {
//       const auto link = sdfModel->LinkByIndex(li);
//       EXPECT_NE(nullptr, link);
//       if (nullptr == link)
//         return nullptr;
//
//       sdf::Link newSdfLink;
//       newSdfLink.SetName(link->Name());
//       newSdfLink.SetRawPose(ResolveSdfPose(link->SemanticPose()));
//       newSdfLink.SetInertial(link->Inertial());
//
//       auto newLink = physModel->ConstructLink(newSdfLink);
//       EXPECT_NE(nullptr, newLink);
//       if (nullptr == newLink)
//         return nullptr;
//
//       linkMap[link] = newLink;
//     }
//   }
//
//   for (auto [sdfLink, physLink] : linkMap)
//   {
//     for (uint64_t ci = 0; ci < sdfLink->CollisionCount(); ++ci)
//     {
//       physLink->ConstructCollision(*sdfLink->CollisionByIndex(ci));
//     }
//   }
//
//   for (auto [sdfModel, physModel] : modelMap)
//   {
//     for (uint64_t ji = 0; ji < sdfModel->JointCount(); ++ji)
//     {
//       const auto sdfJoint = sdfModel->JointByIndex(ji);
//       EXPECT_NE(nullptr, sdfJoint);
//       if (nullptr == sdfJoint)
//         return nullptr;
//
//       std::string resolvedParentLinkName;
//       const auto resolveParentErrors =
//           sdfJoint->ResolveParentLink(resolvedParentLinkName);
//       EXPECT_TRUE(resolveParentErrors.empty()) << resolveParentErrors;
//
//       std::string resolvedChildLinkName;
//       const auto resolveChildErrors =
//         sdfJoint->ResolveChildLink(resolvedChildLinkName);
//       EXPECT_TRUE (resolveChildErrors.empty()) << resolveChildErrors;
//
//       sdf::Joint newSdfJoint;
//       newSdfJoint.SetName(sdfJoint->Name());
//       if (sdfJoint->Axis(0))
//       {
//         newSdfJoint.SetAxis(0, ResolveJointAxis(*sdfJoint->Axis(0)));
//       }
//       if (sdfJoint->Axis(1))
//       {
//         newSdfJoint.SetAxis(1, ResolveJointAxis(*sdfJoint->Axis(1)));
//       }
//       newSdfJoint.SetType(sdfJoint->Type());
//       newSdfJoint.SetRawPose(ResolveSdfPose(sdfJoint->SemanticPose()));
//       newSdfJoint.SetThreadPitch(sdfJoint->ThreadPitch());
//
//       newSdfJoint.SetParentName(resolvedParentLinkName);
//       newSdfJoint.SetChildName(resolvedChildLinkName);
//
//       physModel->ConstructJoint(newSdfJoint);
//     }
//   }
//
//   return world;
// }

/////////////////////////////////////////////////
class SDFFeatures_TEST : public ::testing::TestWithParam<LoaderType>
{
  public: WorldPtr LoadWorld(const std::string &_world)
  {
    switch(this->GetParam())
    {
      case LoaderType::Whole:
        return LoadWorldWhole(_world);
      // case LoaderType::Piecemeal:
      //   return LoadWorldPiecemeal(_world);
      default:
        std::cout << "Unknown LoaderType "
                  << std::underlying_type_t<LoaderType>(this->GetParam())
                  << " Using LoadWorldWhole" << std::endl;
        return LoadWorldWhole(_world);
    }
  }
};
/////////////////////////////////////////////////
// Test that the mujoco plugin loaded all the relevant information correctly.
TEST_P(SDFFeatures_TEST, CheckMujocoData)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kShapesWorld);
  ASSERT_NE(nullptr, world);

}

// Run with different load world functions
// INSTANTIATE_TEST_SUITE_P(LoadWorld, SDFFeatures_TEST,
//                         ::testing::Values(LoaderType::Whole,
//                                           LoaderType::Piecemeal));
INSTANTIATE_TEST_SUITE_P(LoadWorld, SDFFeatures_TEST,
                        ::testing::Values(LoaderType::Whole));
