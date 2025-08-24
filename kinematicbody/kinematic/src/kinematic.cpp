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
            double theta = yamlLink["theta"]["value"].as<double>();
            bool theta_var = yamlLink["theta"]["variable"].as<bool>() ? true : false;
            double* theta_ptr ;

            if(theta_var)
            {
                jointVarList.push_back(theta);
                theta_ptr = &jointVarList.back();
            }
            else
            {   
                theta_ptr = new double(theta);
            }


            //d 
            double d = yamlLink["d"]["value"].as<double>();
            bool d_var = yamlLink["d"]["variable"].as<bool>();
            double* d_ptr;
            if(d_var)
            {
                jointVarList.push_back(d);
                d_ptr = &jointVarList.back();
            }
            else
            {
                d_ptr = new double(d);
            }

            //a
            double a = yamlLink["a"]["value"].as<double>();
            bool a_var = yamlLink["a"]["variable"].as<bool>();
            double* a_ptr ;
            if(a_var)
            {
                jointVarList.push_back(a);
                a_ptr = &jointVarList.back();
            }
            else
            {
                a_ptr = new double(a);
            }
            
            //alfa
            double alfa = yamlLink["a"]["value"].as<double>();
            bool alfa_var = yamlLink["a"]["variable"].as<bool>();
            double* alfa_ptr ;
            if(alfa_var)
            {
                jointVarList.push_back(alfa);
                alfa_ptr = &jointVarList.back();
            }
            else
            {
                alfa_ptr = new double(alfa);
            }

            
            allParam.emplace_back(id, theta_ptr, d_ptr, a_ptr, alfa_ptr);

        }
    }
