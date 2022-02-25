#include "KeyFrame.h"


namespace CS_SLAM{
    
KeyFrame::KeyFrame(){
    mKfPose.hat = Eigen::Vector3d::Zero(3);
    mKfPose.P = Eigen::Matrix3d::Zero(3,3);
    // mvSonarFullScan.resize(0);
    mCimg = cv::Mat::zeros(mCimg.rows, mCimg.cols, CV_32S);
    return ;
}

KeyFrame::~KeyFrame(){};
KeyFrame::KeyFrame(pose kf){
    mKfPose.hat = kf.hat;
    mKfPose.P = kf.P;
    // mvSonarFullScan.resize(0);
    mCimg = cv::Mat::zeros(mCimg.rows, mCimg.cols, CV_32S);
}

KeyFrame::KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP):mKfPose(kfPos,kfPosP){
    // mvSonarFullScan.resize(0);
    mCimg = cv::Mat::zeros(mCimg.rows, mCimg.cols, CV_32S);
}

void KeyFrame::SetPose(pose kfPose){
    mKfPose = kfPose;
};

pose KeyFrame::GetPose(){
    // std::cout<<"Try to "<<mKfPose.hat<<mKfPose.P<<std::endl;
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

// const std::vector<Eigen::VectorXd>& KeyFrame::GetSonarMeasurements(){
//     return mvFsm;
// }

const bool KeyFrame::HaveSonarFullScan(){
    return mvSonarFullScan.size()>0 && mvSonarFullScan.size()<500;
}

const cv::Mat& KeyFrame::GetCameraImage(){
    return mCimg;
}

const bool KeyFrame::HaveCameraImage(){
    return mCimg.data!=nullptr;
}

void KeyFrame::Print(){
    // std::cout<<"KeyFrame info:";
    mKfPose.Print();
    // std::cout<<std::endl;
}

void KeyFrame::LoadCameraImg(std::string filename){
    mCimg = cv::imread(filename);
    if(mCimg.data == nullptr){
        std::cerr<<"camera image not exist"<<std::endl;
        return;
    }
    // std::cout<<"load ok"<<std::endl;
}

void KeyFrame::Transform(motion transform){
    mKfPose = mKfPose.compound(transform);
    return ;
}

unsigned long long KeyFrame::GetTimeStamp(){
    return mTimeStamp;
}

void KeyFrame::SetTimeStamp(unsigned long long timestamp){
    mTimeStamp = timestamp;
}

}

