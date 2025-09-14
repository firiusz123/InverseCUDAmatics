
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "yamlloader.h"
#include "robotmodel.h"
#include <Eigen/Dense>

class YamlLoader
{
    private:
    std::string configPath;
    YAML::Node config ;
    YAML::Node links ;
/********************************************************* */

    public:
    explicit YamlLoader(const std::string& filepath);
    bool LoadModel(RobotModel& model);    
    
    
};