#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>


class Link
{
    private:
    int id ;
    double* theta_ptr ;
    double* d_ptr ;
    double* a_ptr ;
    double* alfa_ptr ;
    public:
    Link(int id , double* theta , double* d , double* a , double* alfa) : id(id) , theta_ptr(theta) , d_ptr(d) , a_ptr(a) , alfa_ptr(alfa)
    {
        std::cout << "link "<<id << " created" <<'\n' ;
    }

};


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
    KinematicBody(const std::string path) : configPath(path)
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
                theta_ptr = &theta;
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

            


        }
    }
};

int main() {
    /*
    // Load the YAML file
    YAML::Node config = YAML::LoadFile("../model/config.yaml");

    // Access the robot node
   
    

    // Access the links
    YAML::Node links = robot["links"];
    for (std::size_t i = 0; i < links.size(); i++) {
        YAML::Node link = links[i];
        std::cout << "Link ID: " << link["id"].as<int>() << "\n";

        std::cout << "  theta: " 
                  << link["theta"]["value"].as<double>() 
                  << " (variable: " 
                  << (link["theta"]["variable"].as<bool>() ? "true" : "false") 
                  << ")\n";

        std::cout << "  d: " 
                  << link["d"]["value"].as<double>() 
                  << " (variable: " 
                  << (link["d"]["variable"].as<bool>() ? "true" : "false") 
                  << ")\n";

        std::cout << "  a: " 
                  << link["a"]["value"].as<double>() 
                  << " (variable: " 
                  << (link["a"]["variable"].as<bool>() ? "true" : "false") 
                  << ")\n";

        std::cout << "  alpha: " 
                  << link["alpha"]["value"].as<double>() 
                  << " (variable: " 
                  << (link["alpha"]["variable"].as<bool>() ? "true" : "false") 
                  << ")\n\n";
    }
    */


    KinematicBody model1("../model/config.yaml");
    return 0;
}
