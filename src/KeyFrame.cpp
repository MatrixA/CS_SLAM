#include "KeyFrame.h"


namespace CS_SLAM{
    
KeyFrame::KeyFrame(){
    mKfPose.hat = Eigen::Vector3d::Zero(3);
    mKfPose.P = Eigen::Matrix3d::Zero(3,3);
    return ;    
}
KeyFrame::~KeyFrame(){};

KeyFrame::KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP):mKfPose(kfPos,kfPosP){}

void KeyFrame::setPos(Eigen::VectorXd kfPos){
    mKfPose.hat = kfPos;
};
void KeyFrame::setPosP(Eigen::MatrixXd kfPosP){
    mKfPose.P = kfPosP;
};
Eigen::VectorXd KeyFrame::GetPos(){
    return mKfPose.hat;
}

Eigen::MatrixXd KeyFrame::GetPosP(){
    return mKfPose.P;
}

void KeyFrame::Transform(motion transform){
    mKfPose = transform.tail2tail(mKfPose);
    return ;
}


}

