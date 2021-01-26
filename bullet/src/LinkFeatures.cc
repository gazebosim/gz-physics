#include "LinkFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalForceInWorld(
    const Identity &_id, const LinearVectorType &_force,
    const LinearVectorType &_position)
{
  btRigidBody* link = this->links.at(_id.id)->link;
  btTransform trans;
  link->getMotionState()->getWorldTransform(trans);
  btVector3 rel_pos = trans.inverse() * convertVec(_position);
  // btVector3 force = trans.inverse() * convertVec(_force);
  link->applyForce(convertVec(_force), rel_pos);
  // link->applyCentralForce(convertVec(_force));
  // ignerr << "force " << force[0] << " " << force[1] << " " << force[2] << std::endl;
}

/////////////////////////////////////////////////
void LinkFeatures::AddLinkExternalTorqueInWorld(
    const Identity &_id, const AngularVectorType &_torque)
{
  btRigidBody* link = this->links.at(_id.id)->link;
  // link->applyTorque(convertVec(_torque));
  // btVector3 force = convertVec(_torque);
  // ignerr << "torque " << force[0] << " " << force[1] << " " << force[2] << std::endl;
}

}
}
}
