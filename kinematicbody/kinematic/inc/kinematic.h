#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "link.h"
#include <Eigen/Dense>

class KinematicBody
{
    private:
    std::string configPath;
    YAML::Node config ;
    YAML::Node model ;
/********************************************************* */
    //array of links
    std::vector<Link> allParam;
    std::vector<float> jointVarList ;

    public:
    KinematicBody(const std::string path);
    Eigen::Vector4d forwardKinematic();
    
    
};