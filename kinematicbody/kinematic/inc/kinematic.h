#include <iostream>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include "link.h"

class KinematicBody
{
    private:
    std::string configPath;
    YAML::Node config ;
    YAML::Node model ;

    //array of links
    std::vector<Link> allParam;
    std::vector<double> jointVarList ;

    public:
    KinematicBody(const std::string path);
};