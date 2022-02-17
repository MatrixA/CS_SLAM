//this file is for testing small functions
#include <iostream>
// #include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <ceres/ceres.h>
#include <chrono>

#include "MSCKF.h"
#include "RandomVector.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frames.h"

using namespace std;

int main(){
    std::cout<<"test tail2tail first"<<std::endl;
    CS_SLAM::pose P1,P2;
    P1 = CS_SLAM::pose(Eigen::Vector3d(0,0,0),0.01*Eigen::MatrixXd::Identity(3,3));
    P2 = CS_SLAM::pose(Eigen::Vector3d(0.9,0.1,1.5),0.01*Eigen::MatrixXd::Identity(3,3));
    P1.tail2tail(P2).Print();

    CS_SLAM::Frames* fdbs = new CS_SLAM::Frames();
    CS_SLAM::KeyFrame a1(Eigen::Vector3d(0,0,0),0.01*Eigen::MatrixXd::Identity(3,3));
    CS_SLAM::KeyFrame a2(Eigen::Vector3d(0.9,0.1,1.5),0.01*Eigen::MatrixXd::Identity(3,3));

    std::vector<CS_SLAM::point> a1s,a2s;
    a1s.push_back(CS_SLAM::point(Eigen::Vector2d(1,2),Eigen::MatrixXd::Identity(2,2)));
    a1s.push_back(CS_SLAM::point(Eigen::Vector2d(1,1),Eigen::MatrixXd::Identity(2,2)));
    a1s.push_back(CS_SLAM::point(Eigen::Vector2d(2,1),Eigen::MatrixXd::Identity(2,2)));
    a1s.push_back(CS_SLAM::point(Eigen::Vector2d(3,1),Eigen::MatrixXd::Identity(2,2)));

    a2s.push_back(CS_SLAM::point(Eigen::Vector2d(1,0),Eigen::MatrixXd::Identity(2,2)));
    a2s.push_back(CS_SLAM::point(Eigen::Vector2d(2,0),Eigen::MatrixXd::Identity(2,2)));
    a2s.push_back(CS_SLAM::point(Eigen::Vector2d(1,-1),Eigen::MatrixXd::Identity(2,2)));
    a2s.push_back(CS_SLAM::point(Eigen::Vector2d(1,-2),Eigen::MatrixXd::Identity(2,2)));

    std::cout<<"kf ok"<<std::endl;

    a1.SetSonarFullScan(a1s);
    a2.SetSonarFullScan(a2s);
    fdbs->add(a1);
    fdbs->add(a2);
    
    
    CS_SLAM::LoopClosing loopClosing(fdbs);
    std::cout<<"full scan ok"<<std::endl;
    CS_SLAM::KeyFrame* x = fdbs->GetKeyFrameByID(0);
    CS_SLAM::KeyFrame* y = fdbs->GetKeyFrameByID(1);
    std::cout<<x->GetPose().hat<<"----"<<y->GetPose().hat<<std::endl;
    CS_SLAM::motion estimate = loopClosing.ScanMatching(y,x);
    estimate.Print();
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