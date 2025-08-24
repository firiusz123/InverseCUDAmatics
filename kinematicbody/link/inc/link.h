#include <iostream>
#include <Eigen/Dense>

class Link
{
    private:
    int id ;
    double* theta_ptr ;
    double* d_ptr ;
    double* a_ptr ;
    double* alfa_ptr ;
    public:
    Link(int id , double* theta , double* d , double* a , double* alfa);
    void valueSet(int id , double* theta , double* d , double* a , double* alfa);
    Eigen::Matrix4d operator*(const Link& other) const;

};