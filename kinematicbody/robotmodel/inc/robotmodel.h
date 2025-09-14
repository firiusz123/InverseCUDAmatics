#pragma once 
#include <iostream>
#include <Eigen/Dense>
#include <vector>


class RobotModel
{
    /**************************************************/
    public:
    std::vector<int> id;
    std::vector<float> theta ,d ,a ,alfa ;
    std::vector<bool> var_theta , var_d , var_a ,var_alfa ;

    std::vector<float*> variable_ptrs;

    std::vector<float> linkCoordinates;

    Eigen::Matrix4d dhTransform = Eigen::Matrix4d::Identity();
    Eigen::Vector4d EndLinkCord;
    /**************************************************/
    bool collectVariables();
    const Eigen::Matrix4d& getDHTransform();
    
    int dof() const { return variable_ptrs.size(); }
};