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
    Eigen::Vector4d origin(0, 0, 0, 1);

    size_t size = id.size();
    dhCumulative.clear();
    linkCoordinates.clear();
    linkCoordinates.reserve(size);
    dhCumulative.reserve(size);

    for(size_t i = 0 ; i < size; i++)
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
        EndLinkCord = dhTransform * origin;
        linkCoordinates.push_back(EndLinkCord);
        dhCumulative.push_back(dhTransform);
    }

    return dhTransform;
}

void RobotModel::variableSetter(const std::vector<float> newValues)
{
    size_t n =variable_ptrs.size();
    if(newValues.size() != n){throw std::runtime_error("Mismatch in number of variable parameters");}
    for(size_t i = 0 ; i < n ; i++)
    {
        *variable_ptrs[i] = newValues[i];
    }
}

Eigen::MatrixXd RobotModel::computeNumericalJacobian(float delta)
{
    int size = variable_ptrs.size();
    //jacobian just for the 3 dimensions for now no need for ortientation
    Eigen::MatrixXd J(3,size);
    Eigen::Vector3d p0 = EndLinkCord.head<3>();

    for(int i = 0 ; i < size ; i++)
    {   
        float original = *variable_ptrs[i];

        *variable_ptrs[i] = original + delta;

        getDHTransform();

        Eigen::Vector3d p1 = EndLinkCord.head<3>();
        J.col(i) = (p1 - p0) / delta ;

        *variable_ptrs[i] = original;
        getDHTransform();
    }

    return J;
}
Eigen::MatrixXd RobotModel::computeGeometricalJacobian()
{
    size_t size = id.size();
    //jacobian just for the 3 dimensions for now no need for ortientation
    Eigen::MatrixXd J(3,size);
    
    Eigen::Vector3d p0 = {0,0,0};
    Eigen::Vector3d z_i ;
    
    size_t col = 0;
    for(int i = 0 ; i < size ; i++)
    {
        Eigen::Vector3d p1 = linkCoordinates[i].head<3>();
        z_i = dhCumulative[i].block<3,1>(0,2);


        if(var_theta[i] || var_alfa[i]){J.col(col++) =z_i.cross(p1 - p0);}
        if(var_a[i] || var_d[i]){J.col(col++) = z_i;}

        p0 = p1;
    }
    return J;
}

