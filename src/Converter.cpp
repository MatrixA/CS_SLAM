#include "Converter.h"

namespace CS_SLAM{

// static KeyFrame Converter::EKFX2X(Eigen::VectorXd hat, Eigen::MatrixXd p){
//     Eigen::VectorXd x_hat = Eigen::Vector3d(hat(0),hat(1),hat(3));
//     Eigen::Matrix3d x_p;
//     x_p.block(0,0,2,2)=p.block(0,0,2,2);x_p.block(0,2,2,1)=p.block(0,3,2,1);
//     x_p.block(2,0,1,2)=p.block(3,0,1,2);x_p(2,2)=p(3,3);
//     return KeyFrame(x_hat,x_p);
// }
bool Converter::isRotationMatrix(cv::Mat &R){
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    
    return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

cv::Vec3f Converter::RotationMatrixToEulerAngles(cv::Mat &R){
    // assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6;
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

motion Converter::FusionMotions(motion a, motion b, double alpha){
    return motion(alpha*a.hat+(1-alpha)*b.hat,a.P);
}

}//namespace CS_SLAM