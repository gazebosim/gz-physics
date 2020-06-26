/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/plugin/Loader.hh>

#include <ignition/common/MeshManager.hh>

#include <ignition/math/eigen3/Conversions.hh>

#include <ignition/physics/Joint.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/RevoluteJoint.hh>

#include "EntityManagementFeatures.hh"
#include "JointFeatures.hh"
#include "KinematicsFeatures.hh"
#include "ShapeFeatures.hh"

#include "test/Utils.hh"

struct TestFeatureList : ignition::physics::FeatureList<
    ignition::physics::dartsim::EntityManagementFeatureList,
    ignition::physics::dartsim::JointFeatureList,
    ignition::physics::dartsim::KinematicsFeatureList,
    ignition::physics::dartsim::ShapeFeatureList
> { };

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(DartsimPluginLib());

  ignition::plugin::PluginPtr dartsim =
      loader.Instantiate("ignition::physics::dartsim::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  ASSERT_NE(nullptr, engine);

  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("empty world", world->GetName());
  EXPECT_EQ(engine, world->GetEngine());

  auto model = world->ConstructEmptyModel("empty model");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ("empty model", model->GetName());
  EXPECT_EQ(world, model->GetWorld());
  EXPECT_NE(model, world->ConstructEmptyModel("dummy"));

  auto link = model->ConstructEmptyLink("empty link");
  ASSERT_NE(nullptr, link);
  EXPECT_EQ("empty link", link->GetName());
  EXPECT_EQ(model, link->GetModel());
  EXPECT_NE(link, model->ConstructEmptyLink("dummy"));

  auto joint = link->AttachRevoluteJoint(nullptr);
  EXPECT_NEAR((Eigen::Vector3d::UnitX() - joint->GetAxis()).norm(), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(0.0, joint->GetPosition(0));

  joint->SetAxis(Eigen::Vector3d::UnitZ());
  EXPECT_NEAR((Eigen::Vector3d::UnitZ() - joint->GetAxis()).norm(), 0.0, 1e-6);

  auto child = model->ConstructEmptyLink("child link");

  const std::string boxName = "box";
  const Eigen::Vector3d boxSize(0.1, 0.2, 0.3);
  auto box = link->AttachBoxShape(boxName, boxSize);
  EXPECT_EQ(boxName, box->GetName());
  EXPECT_NEAR((boxSize - box->GetSize()).norm(), 0.0, 1e-6);

  EXPECT_EQ(1u, link->GetShapeCount());
  auto boxCopy = link->GetShape(0u);
  EXPECT_EQ(box, boxCopy);


  auto prismatic = child->AttachPrismaticJoint(
        link, "prismatic", Eigen::Vector3d::UnitZ());
  const double zPos = 2.5;
  const double zVel = 9.1;
  const double zAcc = 10.2;
  prismatic->SetPosition(0, zPos);
  prismatic->SetVelocity(0, zVel);
  prismatic->SetAcceleration(0, zAcc);

  const ignition::physics::FrameData3d childData =
      child->FrameDataRelativeToWorld();

  const Eigen::Vector3d childPosition = childData.pose.translation();
  EXPECT_DOUBLE_EQ(0.0, childPosition.x());
  EXPECT_DOUBLE_EQ(0.0, childPosition.y());
  EXPECT_DOUBLE_EQ(zPos, childPosition.z());

  const Eigen::Vector3d childVelocity = childData.linearVelocity;
  EXPECT_DOUBLE_EQ(0.0, childVelocity.x());
  EXPECT_DOUBLE_EQ(0.0, childVelocity.y());
  EXPECT_DOUBLE_EQ(zVel, childVelocity.z());

  const Eigen::Vector3d childAcceleration = childData.linearAcceleration;
  EXPECT_DOUBLE_EQ(0.0, childAcceleration.x());
  EXPECT_DOUBLE_EQ(0.0, childAcceleration.y());
  EXPECT_DOUBLE_EQ(zAcc, childAcceleration.z());

  const double yPos = 11.5;
  Eigen::Isometry3d childSpherePose = Eigen::Isometry3d::Identity();
  childSpherePose.translate(Eigen::Vector3d(0.0, yPos, 0.0));
  auto sphere = child->AttachSphereShape("child sphere", 1.0, childSpherePose);

  const ignition::physics::FrameData3d sphereData =
      sphere->FrameDataRelativeToWorld();

  const Eigen::Vector3d spherePosition = sphereData.pose.translation();
  EXPECT_DOUBLE_EQ(0.0, spherePosition.x());
  EXPECT_DOUBLE_EQ(yPos, spherePosition.y());
  EXPECT_DOUBLE_EQ(zPos, spherePosition.z());

  const Eigen::Vector3d sphereVelocity = sphereData.linearVelocity;
  EXPECT_DOUBLE_EQ(0.0, sphereVelocity.x());
  EXPECT_DOUBLE_EQ(0.0, sphereVelocity.y());
  EXPECT_DOUBLE_EQ(zVel, sphereVelocity.z());

  const Eigen::Vector3d sphereAcceleration = sphereData.linearAcceleration;
  EXPECT_DOUBLE_EQ(0.0, sphereAcceleration.x());
  EXPECT_DOUBLE_EQ(0.0, sphereAcceleration.y());
  EXPECT_DOUBLE_EQ(zAcc, sphereAcceleration.z());

  const ignition::physics::FrameData3d relativeSphereData =
      sphere->FrameDataRelativeTo(*child);
  const Eigen::Vector3d relativeSpherePosition =
      relativeSphereData.pose.translation();
  EXPECT_DOUBLE_EQ(0.0, relativeSpherePosition.x());
  EXPECT_DOUBLE_EQ(yPos, relativeSpherePosition.y());
  EXPECT_DOUBLE_EQ(0.0, relativeSpherePosition.z());

  auto meshLink = model->ConstructEmptyLink("mesh_link");
  meshLink->AttachFixedJoint(child, "fixed");

  const std::string meshFilename = IGNITION_PHYSICS_RESOURCE_DIR "/chassis.dae";
  auto &meshManager = *ignition::common::MeshManager::Instance();
  auto *mesh = meshManager.Load(meshFilename);

  auto meshShape = meshLink->AttachMeshShape("chassis", *mesh);
  const auto originalMeshSize = mesh->Max() - mesh->Min();
  const auto meshShapeSize = meshShape->GetSize();

  // Note: dartsim uses assimp for storing mesh data, and assimp by default uses
  // single floating point precision (instead of double precision), so we can't
  // expect these values to be exact.
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(originalMeshSize[i], meshShapeSize[i], 1e-6);

  EXPECT_NEAR(meshShapeSize[0], 0.5106, 1e-4);
  EXPECT_NEAR(meshShapeSize[1], 0.3831, 1e-4);
  EXPECT_NEAR(meshShapeSize[2], 0.1956, 1e-4);

  const ignition::math::Pose3d pose(0, 0, 0.2, 0, 0, 0);
  const ignition::math::Vector3d scale(0.5, 1.0, 0.25);
  auto meshShapeScaled = meshLink->AttachMeshShape("small_chassis", *mesh,
                          ignition::math::eigen3::convert(pose),
                          ignition::math::eigen3::convert(scale));
  const auto meshShapeScaledSize = meshShapeScaled->GetSize();

  // Note: dartsim uses assimp for storing mesh data, and assimp by default uses
  // single floating point precision (instead of double precision), so we can't
  // expect these values to be exact.
  for (std::size_t i = 0; i < 3; ++i)
    EXPECT_NEAR(originalMeshSize[i] * scale[i], meshShapeScaledSize[i], 1e-6);

  EXPECT_NEAR(meshShapeScaledSize[0], 0.2553, 1e-4);
  EXPECT_NEAR(meshShapeScaledSize[1], 0.3831, 1e-4);
  EXPECT_NEAR(meshShapeScaledSize[2], 0.0489, 1e-4);
}

TEST(EntityManagement_TEST, RemoveEntities)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(DartsimPluginLib());

  ignition::plugin::PluginPtr dartsim =
      loader.Instantiate("ignition::physics::dartsim::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  ASSERT_NE(nullptr, engine);

  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
  auto model = world->ConstructEmptyModel("empty model");
  ASSERT_NE(nullptr, model);

  auto modelAlias = world->GetModel(0);

  model->Remove();
  EXPECT_TRUE(model->Removed());
  EXPECT_TRUE(modelAlias->Removed());
  EXPECT_EQ(nullptr, world->GetModel(0));
  EXPECT_EQ(nullptr, world->GetModel("empty model"));
  EXPECT_EQ(0ul, world->GetModelCount());

  // Calling GetName shouldn't throw
  EXPECT_EQ("empty model", model->GetName());

  auto model2 = world->ConstructEmptyModel("model2");
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ(0ul, model2->GetIndex());
  world->RemoveModel(0);
  EXPECT_EQ(0ul, world->GetModelCount());
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
