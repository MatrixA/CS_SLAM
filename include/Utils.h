#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include "RandomVector.h"

namespace CS_SLAM
{

class Utils
{
public:
    static double MahDistance(const Eigen::VectorXd &ri, const Eigen::MatrixXd &Sigma, const Eigen::VectorXd &ni);
    static Eigen::Vector2d Oplus(const Eigen::Vector3d &q,const Eigen::Vector2d &n);
    static Eigen::Vector3d Odot(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
    static std::string TimeStamp2TimeString(unsigned long long timestamp);
    // static RandomVector CompoundP(RandomVector q, RandomVector b);
private:
    static const int daysPerMonth[13];
};

}// namespace ORB_SLAM

#endif // UTILS_H
