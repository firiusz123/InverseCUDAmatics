#include "kinematic.h"

KinematicBody::KinematicBody(RobotModel& model) : model(model){}
Eigen::Vector4d KinematicBody::forwardKinematic()
{
  model.getDHTransform();
  return model.EndLinkCord;
}







