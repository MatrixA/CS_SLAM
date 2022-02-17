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

int main(){
    Eigen::VectorXd a=Eigen::VectorXd::Zero(8);
    a(4)=0.1;
    a(5)=0.1;
    a(6)=0.1;
    a(7)=0.05;
    CS_SLAM::EKF ekf(a);
    ekf.prediction(5);
    std::cout<<ekf.GetX()<<std::endl;
    std::cout<<ekf.GetP()<<std::endl;
    return 0;
}