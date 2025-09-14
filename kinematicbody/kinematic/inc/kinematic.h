#pragma once
#include <iostream>
#include <vector>
#include "robotmodel.h"
#include <Eigen/Dense>

class KinematicBody
{
    private:
    RobotModel& model;
/********************************************************* */
    //array of links

    public:
    KinematicBody(RobotModel& model );
    Eigen::Vector4d forwardKinematic();
    int dof() const;
    void setJointAngles(const std::vector<float>& q);
    bool inverseKinematic(const Eigen::Vector3d& target , int iter_limit , double tolerance , double alpha);
    Eigen::MatrixXd computeJacobian(double eps);
    
    
    
};