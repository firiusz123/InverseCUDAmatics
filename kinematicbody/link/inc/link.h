#include <iostream>
#include <Eigen/Dense>

class Link
{
    private:
    int id ;
    float* theta_ptr ;
    float* d_ptr ;
    float* a_ptr ;
    float* alfa_ptr ;
    Eigen::Matrix4d dhTransform;
    Eigen::Vector4d EndLinkCord;
 /************************************************************************************** */
    public:
    Link(int id , float* theta , float* d , float* a , float* alfa);
    void valueSet(int id , float* theta , float* d , float* a , float* alfa);
    Eigen::Matrix4d operator*(const Link& other) const;
    const Eigen::Matrix4d& getDHTransform() const {return dhTransform;}
    void setEndLinkCord(const Eigen::Vector4d x){this->EndLinkCord = x;}

};