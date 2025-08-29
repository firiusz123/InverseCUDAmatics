#include "kinematic.h"

KinematicBody::KinematicBody(const std::string path) : configPath(path)
    {
        try
        {
        this->config = YAML::LoadFile(path);
        }
        catch (const YAML::Exception& e){std::cout << "wrong path" << '\n' ;}
        
        if (config["model"] && config["model"]["name"]) 
        {
            this->model = config["model"];
            std::cout << "Robot name: " << model["name"].as<std::string>() << "\n\n";
            
        }
        YAML::Node links = model["links"];
        for (std::size_t i = 0; i < links.size(); i++)
        {
            YAML::Node yamlLink = links[i];

            int id = yamlLink["id"].as<int>();


            //theta 
            float theta = yamlLink["theta"]["value"].as<float>();
            bool theta_var = yamlLink["theta"]["variable"].as<bool>() ? true : false;
            float* theta_ptr ;

            if(theta_var)
            {
                jointVarList.push_back(theta);
                theta_ptr = &jointVarList.back();
            }
            else
            {   
                theta_ptr = new float(theta);
            }


            //d 
            float d = yamlLink["d"]["value"].as<float>();
            bool d_var = yamlLink["d"]["variable"].as<bool>();
            float* d_ptr;
            if(d_var)
            {
                jointVarList.push_back(d);
                d_ptr = &jointVarList.back();
            }
            else
            {
                d_ptr = new float(d);
            }

            //a
            float a = yamlLink["a"]["value"].as<float>();
            bool a_var = yamlLink["a"]["variable"].as<bool>();
            float* a_ptr ;
            if(a_var)
            {
                jointVarList.push_back(a);
                a_ptr = &jointVarList.back();
            }
            else
            {
                a_ptr = new float(a);
            }
            
            //alfa
            float alfa = yamlLink["a"]["value"].as<float>();
            bool alfa_var = yamlLink["a"]["variable"].as<bool>();
            float* alfa_ptr ;
            if(alfa_var)
            {
                jointVarList.push_back(alfa);
                alfa_ptr = &jointVarList.back();
            }
            else
            {
                alfa_ptr = new float(alfa);
            }

            
            allParam.emplace_back(id, theta_ptr, d_ptr, a_ptr, alfa_ptr);

        }
}
Eigen::Vector4d KinematicBody::forwardKinematic()
{
    Eigen::Vector4d point_local(0, 0, 0, 1);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (size_t i = 0 ; i < (allParam.size()); i++)
    {
        transform = transform * allParam[i].getDHTransform();
        allParam[i].setEndLinkCord(transform * point_local);
    }
    
    return transform * point_local ;
}
