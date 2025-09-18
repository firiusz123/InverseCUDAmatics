#pragma once
#include <iostream>
#include <vector>
#include "robotmodel.h"
#include <Eigen/Dense>

class KinematicBody
{
    private:
    RobotModel& model;
    Eigen::Vector3d targetParams;
/********************************************************* */
    //array of links

    public:
    KinematicBody(RobotModel& model );
    Eigen::Vector4d forwardKinematic();
    bool inverseKinematic(const Eigen::Vector3d& target , float e = 1e-2 , int iter_limit = 100 , float alpha = 1e-1 );

    
    
    
    
};