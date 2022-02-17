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
    CS_SLAM::MSCKF MSCKF;
    MSCKF.Initialize(CS_SLAM::motion(Eigen::Vector3d(1,1,2),Eigen::MatrixXd::Zero(3,3)));
    MSCKF.Print();

    MSCKF.Prediction(CS_SLAM::motion(Eigen::Vector3d(0.1,0.2,1),Eigen::MatrixXd::Zero(3,3)));
    MSCKF.Print();

    MSCKF.Prediction(CS_SLAM::motion(Eigen::Vector3d(-0.1,-0.2,1),Eigen::MatrixXd::Zero(3,3)));
    MSCKF.Print();
    
    return 0;
}


// #include <gtest/gtest.h>



// TEST(MyslamTest, OKFINE){
//     double a=3.11;
//     EXPECT_NEAR(a,3.12,0.03);
// }


// int main(int argc, char **argv){
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }