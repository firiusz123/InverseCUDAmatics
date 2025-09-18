#include "kinematic.h"

KinematicBody::KinematicBody(RobotModel& model) : model(model){}
Eigen::Vector4d KinematicBody::forwardKinematic()
{
  model.getDHTransform();
  return model.EndLinkCord;
}
bool KinematicBody::inverseKinematic(const Eigen::Vector3d& target , float e , int iter_limit , float alpha )
{
  size_t paramSize = model.variable_ptrs.size();
  Eigen::VectorXd q(paramSize);
  
  for (size_t i = 0; i < paramSize; i++) 
  q[i] = *model.variable_ptrs[i];

  
  
  for(int i = 0 ; i < iter_limit ; i++)
  {

    model.getDHTransform();
    model.collectVariables();
    Eigen::Vector3d p0 = model.EndLinkCord.head<3>();
    Eigen::Vector3d p_delta = target - p0 ;
    std::cout << p_delta <<'\n';

    Eigen::MatrixXd J = model.computeNumericalJacobian();
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::VectorXd dq = J_pinv * p_delta;
    q += alpha * dq;

    std::vector<float> newParams(q.data(), q.data() + q.size());
    model.variableSetter(newParams);
    if(p_delta.norm() < e)
    {
      std::cout << "Converged in " << i+1 << " iterations.\n";
      std::cout << "Final parameters:\n" << q.transpose() << "\n";
      return true;
    }
  }

  std::cout << "IK did not converge within " << iter_limit << " iterations.\n";

  return false;
  
}







