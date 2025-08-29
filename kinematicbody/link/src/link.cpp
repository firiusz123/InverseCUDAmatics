#include "link.h"

Link::Link(int id , float* theta , float* d , float* a , float* alfa) : id(id) , theta_ptr(theta) , d_ptr(d) , a_ptr(a) , alfa_ptr(alfa)
    {
        
        float cosTheta = std::cos(*theta);
        float cosAlfa = std::cos(*alfa);
        float sinTheta = std::sin(*theta);
        float sinAlfa = std::sin(*alfa);

        dhTransform << cosTheta , - sinTheta * cosAlfa , sinTheta * sinAlfa ,(*a) * cosTheta
        , sinTheta , cosTheta * cosAlfa , -cosTheta * sinAlfa , (*a)*sinTheta
        , 0.0 , sinAlfa , cosAlfa , (*d)
        , 0.0 , 0.0 , 0.0 , 1.0;


        std::cout << "link "<<id << " created" <<'\n' ;
        for (int i = 0; i < 4; i++) 
        {
            for (int j = 0; j < 4; j++) {
                std::cout << dhTransform(i,j) << " ";
            }
            std::cout << "\n";

        }
    }
/***********************************************************************************************************************************************/
void Link::valueSet(int new_id,
     float* new_theta,
      float* new_d,
       float* new_a,
        float* new_alfa)
{
    this->id = new_id;
    this->theta_ptr = new_theta;
    this->d_ptr = new_d;
    this->a_ptr = new_a;
    this->alfa_ptr =new_alfa;
}
/******************************************************************************************************************************************** */
Eigen::Matrix4d Link::operator*(const Link& other) const
{
    Eigen::Matrix4d A ;
    //A << std::cos(this->theta_ptr) << - std::sin(this->theta_ptr) << 0 << thisa_ptr;
    A << std::cos(*this->theta_ptr);
    Eigen::Matrix4d B ;
    
    return A;
}
/******************************************************************************************************************************************** */