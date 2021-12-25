#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace CS_SLAM
{

class Utils
{
public:
    static double MahDistance(const Eigen::Vector3d &ri, const Eigen::Matrix3d &Sigma, const Eigen::Vector3d &ni);
    static Eigen::Vector2d Oplus(const Eigen::Vector3d &q,const Eigen::Vector2d &n);
    static Eigen::Vector3d Odot(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

};

}// namespace ORB_SLAM

#endif // UTILS_H
