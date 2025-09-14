#include "yamlloader.h"

YamlLoader::YamlLoader(const std::string& filepath) : configPath(filepath){}
static void parseParam(const YAML::Node& node , std::vector<float>& values , std::vector<bool>& vars)
{
    values.push_back(node["value"].as<float>());
    vars.push_back(node["variable"].as<bool>());
}
bool YamlLoader::LoadModel(RobotModel& model)
{
     try {
            config = YAML::LoadFile(configPath);

            if (!config["model"] || !config["model"]["links"]) {
                std::cerr << "Invalid YAML structure\n";
                return false;
            }

            links = config["model"]["links"];

            size_t n = links.size();
            model.id.reserve(n);
            model.theta.reserve(n);
            model.d.reserve(n);
            model.a.reserve(n);
            model.alfa.reserve(n);
            model.var_theta.reserve(n);
            model.var_d.reserve(n);
            model.var_a.reserve(n);
            model.var_alfa.reserve(n);

            for (std::size_t i = 0; i < links.size(); i++) {
                YAML::Node yamlLink = links[i];

                // ID
                model.id.push_back(yamlLink["id"].as<int>());

                // θ
                parseParam(yamlLink["theta"], model.theta, model.var_theta);

                // d
                parseParam(yamlLink["d"] , model.d ,model.var_d);

                // a
                parseParam(yamlLink["a"] , model.a ,model.var_a);
                // α
                parseParam(yamlLink["alfa"] , model.alfa ,model.var_alfa);
            }
            model.collectVariables();
            std::cout << "Loaded robot with " << model.id.size() << " links\n";
            return true;
        }
        catch (const YAML::Exception& e) {
            std::cerr << "YAML parse error: " << e.what() << "\n";
            return false;
        }
}