#include "KeyFrame.h"


namespace CS_SLAM{
    
KeyFrame::KeyFrame(){
    mKfPose.hat = Eigen::Vector3d::Zero(3);
    mKfPose.P = Eigen::Matrix3d::Zero(3,3);
    mvSonarFullScan.resize(0);
    return ;
}

KeyFrame::~KeyFrame(){};
KeyFrame::KeyFrame(pose kf){
    mKfPose.hat = kf.hat;
    mKfPose.P = kf.P;
}

KeyFrame::KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP):mKfPose(kfPos,kfPosP){}

void KeyFrame::SetPose(pose kfPose){
    mKfPose = kfPose;
};

pose KeyFrame::GetPose(){
    return mKfPose;
}

void KeyFrame::SetSonarFullScan(std::vector<point> fs){
    mvSonarFullScan.assign(fs.begin(),fs.end());
    // std::cout<<"check "<<mvSonarFullScan[1].hat(0)<<","<<mvSonarFullScan[1].hat(1)<<std::endl;
    // std::cout<<"verse "<<fs[1].hat(0)<<","<<fs[1].hat(1)<<std::endl;
}

const std::vector<point>& KeyFrame::GetSonarFullScan(){
    return mvSonarFullScan;
}

void KeyFrame::SetSonarMeasurements(const std::vector<Eigen::VectorXd>& fsm){
    mvFsm.assign(fsm.begin(),fsm.end());
}

const std::vector<Eigen::VectorXd>& KeyFrame::GetSonarMeasurements(){
    return mvFsm;
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

