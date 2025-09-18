#include "yamlloader.h"
#include "robotmodel.h"
#include "kinematic.h"
#include <iostream>
#include <vector>


int main()
{
    std::vector<float> newParams = {M_PI/2 , 0.0 , 0.0};
    
    // 1. Load robot model from YAML
    std::string filepath = "../data/config.yaml";
    RobotModel robot;
    YamlLoader loader(filepath);

    if (!loader.LoadModel(robot))
    {
        std::cerr << "Failed to load robot model.\n";
        return -1;
    }

    KinematicBody model1(robot);
    Eigen::Vector4d endEffector = model1.forwardKinematic();

    std::cout << "Initial pose:\n" << endEffector.transpose() << "\n";
    //robot.variableSetter(newParams);
    endEffector = model1.forwardKinematic();
    Eigen::MatrixXd J = robot.computeNumericalJacobian();
    std::cout << "final pose:\n" << endEffector.transpose() << "\n";
    std::cout << "jacobian :\n" << J << "\n";

    Eigen::Vector3d t = {0 , 1.8 , 0};
    model1.inverseKinematic(t);
    return 0;
}