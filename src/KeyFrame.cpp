#include "KeyFrame.h"


namespace CS_SLAM{
    
KeyFrame::KeyFrame(){
    mKfPose.hat = Eigen::Vector3d::Zero(3);
    mKfPose.P = Eigen::Matrix3d::Zero(3,3);
    mvSonarFullScan.resize(0);
    return ;
}
KeyFrame::~KeyFrame(){};

KeyFrame::KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP):mKfPose(kfPos,kfPosP){}

void KeyFrame::SetPos(Eigen::VectorXd kfPos){
    mKfPose.hat = kfPos;
};
void KeyFrame::SetPosP(Eigen::MatrixXd kfPosP){
    mKfPose.P = kfPosP;
};
Eigen::VectorXd KeyFrame::GetPos(){
    return mKfPose.hat;
}

Eigen::MatrixXd KeyFrame::GetPosP(){
    return mKfPose.P;
}

void KeyFrame::SetSonarFullScan(std::vector<point> fs){
    mvSonarFullScan.assign(fs.begin(),fs.end());
}

std::vector<point> KeyFrame::GetSonarFullScan(){
    return mvSonarFullScan;
}

void KeyFrame::Print(){
    std::cout<<"KeyFrame info:";
    mKfPose.Print();
    std::cout<<std::endl;
}

void KeyFrame::Transform(motion transform){
    mKfPose = transform.tail2tail(mKfPose);
    return ;
}


}

