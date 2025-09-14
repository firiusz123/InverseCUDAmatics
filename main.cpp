#include "yamlloader.h"
#include "robotmodel.h"
#include "kinematic.h"
#include <iostream>
#include <vector>

int main()
{
    // 1. Load robot model from YAML
    std::string filepath = "../data/config.yaml";
    RobotModel robot;
    YamlLoader loader(filepath);

    if (!loader.LoadModel(robot))
    {
        std::cerr << "Failed to load robot model.\n";
        return -1;
    }

    // 2. Create KinematicBody
    KinematicBody model1(robot);

    // 3. Compute initial FK
    Eigen::Vector4d endEffector = model1.forwardKinematic();
    std::cout << "Initial pose:\n" << endEffector.transpose() << "\n";

    return 0;
}