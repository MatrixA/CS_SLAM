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
    // std::cout<<a.hat<<" with "<<b.hat<<" ||| "<<alpha<< std::endl;
    // std::cout<<a.hat<<" : "<<b.hat<<" ||| alpha"<<std::endl;
    return motion(alpha*a.hat+(1-alpha)*b.hat,a.P);
}

void Converter::CameraToDVL(cv::Mat R, cv::Mat t, cv::Mat& RDvl, cv::Mat& tDvl){
    Eigen::MatrixXd Ro(3,3),to(3,1);
    cv::cv2eigen(R,Ro);
    cv::cv2eigen(t,to);
    
    RDvl.at<double>(0,0)=Ro(0,0); RDvl.at<double>(0,0)=Ro(0,2); RDvl.at<double>(0,0)=Ro(0,1);
    RDvl.at<double>(0,0)=Ro(2,0); RDvl.at<double>(0,0)=Ro(2,2); RDvl.at<double>(0,0)=Ro(2,1);
    RDvl.at<double>(0,0)=Ro(1,0); RDvl.at<double>(0,0)=Ro(1,2); RDvl.at<double>(0,0)=Ro(1,1);

    tDvl.at<double>(0,0)=0.02*Ro(0,1)-0.26*Ro(0,0)+to(0)+0.26;
    tDvl.at<double>(1,0)=0.02*Ro(2,1)-0.26*Ro(2,0)+to(2);
    tDvl.at<double>(2,0)=0.02*Ro(1,1)-0.26*Ro(1,0)+to(1)-0.02;
}

}//namespace CS_SLAM