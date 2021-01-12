#include <ignition/common/Console.hh>
#include "KinematicsFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld())
  {
    ignerr << "Given a FrameID belonging to the world. This should not be "
           << "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  const auto linkID = _id.ID();

  if (this->links.find(linkID) == this->links.end())
  {
    ignerr << "Given a FrameID not belonging to a link.\n";
    return data;
  }
  const auto &linkInfo = this->links.at(linkID);
  const auto &model = linkInfo->link;
  const auto &collisionID = this->link_to_collision.at(linkID);
  const auto &pose = this->collisions.at(collisionID)->pose;

  btTransform trans, collision_tf;
  model->getMotionState()->getWorldTransform(trans);
  const auto poseIsometry = ignition::math::eigen3::convert(pose.Inverse());
  const auto poseTranslation = poseIsometry.translation();
  const auto poseLinear = poseIsometry.linear();
  collision_tf.setOrigin(convertVec(poseTranslation));
  collision_tf.setBasis(convertMat(poseLinear));
  trans = trans * collision_tf;
  const btVector3 pos = trans.getOrigin();
  const btMatrix3x3 mat = trans.getBasis();

  auto eigenMat = convert(mat);
  auto eigenVec = convert(pos);

  data.pose.linear() = eigenMat;
  data.pose.translation() = eigenVec;

  // Add base velocities
  btVector3 omega = model->getAngularVelocity();
  btVector3 vel = model->getLinearVelocity();

  // Transform to world frame
  // const auto matBaseToWorld = btMatrix3x3(model->getWorldToBaseRot()).inverse();
  // omega = matBaseToWorld * omega;
  // vel = matBaseToWorld * vel;

  data.linearVelocity = convert(vel);
  data.angularVelocity = convert(omega);

  // \todo(anyone) compute frame accelerations

  return data;
}

}
}
}
