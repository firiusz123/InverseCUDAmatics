#include "link.h"

Link::Link(int id , double* theta , double* d , double* a , double* alfa) : id(id) , theta_ptr(theta) , d_ptr(d) , a_ptr(a) , alfa_ptr(alfa)
    {
        std::cout << "link "<<id << " created" <<'\n' ;
    }
void Link::valueSet(int new_id,
     double* new_theta,
      double* new_d,
       double* new_a,
        double* new_alfa)
{
    this->id = new_id;
    this->theta_ptr = new_theta;
    this->d_ptr = new_d;
    this->a_ptr = new_a;
    this->alfa_ptr =new_alfa;
}
Eigen::Matrix4d Link::operator*(const Link& other) const
{
    Eigen::Matrix4d A << std::cos(this->theta_ptr) << - std::sin(this->theta_ptr) << 0 <<
    Eigen::Matrix4d B ;
    
    return A;
}
