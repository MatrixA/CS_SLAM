#include "Utils.h"

namespace CS_SLAM
{

double Utils::MahDistance(const Eigen::Vector3d &ri, const Eigen::Matrix3d &Sigma, const Eigen::Vector3d &ni){
/*
输入：向量ri,协方差矩阵，向量ni
输出：ri和ni的马氏距离
*/
    return sqrt((ri - ni).transpose() * Sigma.inverse() * (ri - ni));
}


Eigen::Vector2d Utils::Oplus(const Eigen::Vector3d &q,const Eigen::Vector2d &n){
    /*
        输入:不确定点p和不确定位移q
        输出:位移后的不确定点np
    */
    Eigen::Vector2d np(n(0)*cos(q(2))-n(1)*sin(q(2))+q(0), n(0)*sin(q(2))+n(1)*cos(q(2))+q(1));
    return np;
}

Eigen::Vector3d Odot(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
    return Eigen::Vector3d(a(0)+b(0),a(1)+b(1),b(2));
}


}