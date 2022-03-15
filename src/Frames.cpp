#include "Frames.h"

namespace CS_SLAM{

Frames::Frames(){
    KeyFrameDatabase.resize(0);
    mpLastCameraKeyFrame = nullptr;
}


/**
 * @brief add KeyFrame to FramesDatabase, typeID specify types.
 *        
 * @param kf - KeyFrame to add
 * @param status - 1:camera
 */
void Frames::add(KeyFrame kf, int typeID){
    std::unique_lock<std::mutex> lock(mMutex);
    KeyFrameDatabase.push_back(kf);
    if(typeID==1){
        mpLastCameraKeyFrame = &KeyFrameDatabase.back();
    }else{
        IndOfSonarKeyFrames.push_back(KeyFrameDatabase.size()-1);
    }
    return ;
}

void Frames::erase(int ind){
    std::unique_lock<std::mutex> lock(mMutex);
    KeyFrameDatabase.erase(KeyFrameDatabase.begin()+ind);
    return ;
}

void Frames::clear(){
    std::unique_lock<std::mutex> lock(mMutex);
    KeyFrameDatabase.clear();
    return ;
}

void Frames::clearMap(LocalMap* pMap){
    std::unique_lock<std::mutex> lock(mMutex);
    return ;
}

int Frames::Size(){
    return KeyFrameDatabase.size();
}

KeyFrame* Frames::GetCurrentKeyFrame(){
    std::unique_lock<std::mutex> lock(mMutex);
    return &(KeyFrameDatabase.back());
}

KeyFrame* Frames::GetKeyFrameByID(int id, bool sonar=true){
    std::unique_lock<std::mutex> lock(mMutex);
    if(sonar)return &KeyFrameDatabase[IndOfSonarKeyFrames[id]];
    else return &KeyFrameDatabase[id];
}

KeyFrame* Frames::GetLastCameraKeyFrame(){
    std::unique_lock<std::mutex> lock(mMutex);
    return mpLastCameraKeyFrame;
}

bool Frames::IsInitiliedCam(){
    return mpLastCameraKeyFrame!=nullptr;
}

bool Frames::HaveFrames(){
    return KeyFrameDatabase.size()>0;
}


std::vector<int> Frames::GetOverlaps(KeyFrame kf, int threshold){
/*
输入：当前扫描对应的位姿x_nk，距离阈值threshold
输出：所有相邻扫描对应的位姿id数组
*/
    Eigen::Vector3d x_nk = kf.GetPose().hat;
    std::vector<int> ans;
    for(int i = 0; i < KeyFrameDatabase.size(); i++){
        double dis = (KeyFrameDatabase[i].GetPose().hat - x_nk).norm();
        if(dis < threshold){
            ans.push_back(i);
        }
    }
    return ans;
}

std::vector<int> Frames::GetCurrentOverlaps(int threshold){
    Eigen::Vector3d x_nk= GetCurrentKeyFrame()->GetPose().hat;
    std::vector<int> ans;
    for(int i = 0; i < IndOfSonarKeyFrames.size()-1; i++){
        KeyFrame tmpKF = KeyFrameDatabase[IndOfSonarKeyFrames[i]];
        Eigen::Vector3d tmp = tmpKF.GetPose().hat - x_nk;
        double dis = sqrt(tmp(0)*tmp(0)+tmp(1)*tmp(1));
        // double dis = (KeyFrameDatabase[i].GetPose().hat - x_nk).norm();
        if(dis < threshold){
            ans.push_back(i);
        }
    }
    return ans;
}

void Frames::Init2DFromFile(std::string filename, bool haveObs){
    std::ifstream infile(filename);
    if(!infile.is_open()){
        std::cout<<"Init FrameDatabase From null file"<<std::endl;
        return ;
    }
    std::cout<<"here"<<std::endl;
    if(!haveObs){
        int id;
        double x_, y_, theta_;
        std::string line;
        while(infile>>id>>x_>>y_>>theta_){
            add(KeyFrame(Eigen::Vector3d(x_, y_, theta_),Eigen::Matrix3d::Identity()));
            std::cout<<"add a pose in init "<<id<<" : "<<GetCurrentKeyFrame()->GetPose().hat<<std::endl;
        }
    }
    return ;
}

void Frames::AlterPose(int ind, pose ps){
    // ps.Print();
    KeyFrameDatabase[ind].SetPose(ps);
}


}