#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>
#include<Eigen/Dense>
#include<opencv2/core/eigen.hpp>
#include"RandomVector.h"

// #include "KeyFrame.h"
// #include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
// #include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace CS_SLAM
{

class Converter
{
public:
    // static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    // static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    // static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    // static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    // static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    // static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    // static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    // static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    // static cv::Mat toCvMat(const Eigen::MatrixXd &m);

    // static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    // static cv::Mat tocvSkewMatrix(const cv::Mat &v);

    // static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    // static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    // static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    // static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);
    // static std::vector<float> toQuaternion(const cv::Mat &M);

    // static bool isRotationMatrix(const cv::Mat &R);
    // static std::vector<float> toEuler(const cv::Mat &R);
    // static KeyFrame EKFX2X(Eigen::VectorXd hat, Eigen::MatrixXd p){
    //     Eigen::VectorXd x_hat = Eigen::Vector3d(hat(0),hat(1),hat(3));
    //     Eigen::Matrix3d x_p;
    //     x_p.block(0,0,2,2)=p.block(0,0,2,2);x_p.block(0,2,2,1)=p.block(0,3,2,1);
    //     x_p.block(2,0,1,2)=p.block(3,0,1,2);x_p(2,2)=p(3,3);
    //     return KeyFrame(x_hat,x_p);
    // }
    static bool isRotationMatrix(cv::Mat &R);
    static cv::Vec3f RotationMatrixToEulerAngles(cv::Mat &R);
    static motion FusionMotions(motion a, motion b, double alpha);
    static void CameraToDVL(cv::Mat R, cv::Mat t, cv::Mat& RDvl, cv::Mat& tDvl);
};

}// namespace CS_SLAM

#endif // CONVERTER_H
