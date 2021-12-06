#include "KeyFrame.h"


namespace CS_SLAM{
    
KeyFrame::KeyFrame(){
    kfPose.hat = Eigen::Vector3d::Zero(3);
    kfPose.P = Eigen::Matrix3d::Zero(3,3);
    return ;    
}
KeyFrame::~KeyFrame(){};

KeyFrame::KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP):kfPose(kfPos,kfPosP){}

void KeyFrame::setPos(Eigen::VectorXd kfPos){
    kfPose.hat = kfPos;
};
void KeyFrame::setPosP(Eigen::MatrixXd kfPosP){
    kfPose.P = kfPosP;
};
Eigen::VectorXd KeyFrame::GetPos(){
    return kfPose.hat;
}

Eigen::MatrixXd KeyFrame::GetPosP(){
    return kfPose.P;
}

void KeyFrame::Transform(motion transform){
    kfPose = transform.tail2tail(kfPose);
    return ;
}


}

