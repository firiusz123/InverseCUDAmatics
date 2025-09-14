#include "robotmodel.h"

/******************************************************************************************************************************************** */
// the RobotModel class 
// SoA architecture 

bool RobotModel::collectVariables()
{
    variable_ptrs.clear();
    for(size_t i = 0 ; i < theta.size(); i++)
    {
        if (var_theta[i]) variable_ptrs.push_back(&theta[i]);
        if (var_d[i])   variable_ptrs.push_back(&d[i]);
        if (var_a[i])   variable_ptrs.push_back(&a[i]);
        if (var_alfa[i])    variable_ptrs.push_back(&alfa[i]);
    }
    return !variable_ptrs.empty();
}
const Eigen::Matrix4d& RobotModel::getDHTransform()
{
    
    dhTransform = Eigen::Matrix4d::Identity();

    for(size_t i = 0 ; i < id.size(); i++)
    {
        Eigen::Matrix4d m1;
        float cosTheta = std::cos(theta[i]);
        float cosAlfa = std::cos(alfa[i]);
        float sinTheta = std::sin(theta[i]);
        float sinAlfa = std::sin(alfa[i]);
        
        m1 << cosTheta , - sinTheta * cosAlfa , sinTheta * sinAlfa ,(a[i]) * cosTheta
        , sinTheta , cosTheta * cosAlfa , -cosTheta * sinAlfa , a[i]*sinTheta
        , 0.0 , sinAlfa , cosAlfa , (d[i])
        , 0.0 , 0.0 , 0.0 , 1.0;

        dhTransform *= m1;
    }
    EndLinkCord = dhTransform * Eigen::Vector4d(0, 0, 0, 1);
    return dhTransform;
}