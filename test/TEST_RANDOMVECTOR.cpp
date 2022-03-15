//this file is for testing small functions
#include <iostream>
// #include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
// #include <ceres/ceres.h>
#include <chrono>

#include "MSCKF.h"
#include "RandomVector.h"
#include "KeyFrame.h"

using namespace std;

int main(){
    CS_SLAM::pose a(Eigen::Vector3d(0.5,0,0.5),Eigen::Matrix3d::Zero(3,3));
    CS_SLAM::pose b(Eigen::Vector3d(1,1,0.5),Eigen::Matrix3d::Zero(3,3));
    // a.tail2tail(b).Print();

    (a.compound(CS_SLAM::motion(Eigen::Vector3d(0.918217,0.63787,0),Eigen::Matrix3d::Zero(3,3)))).Print();
    return 0;
}