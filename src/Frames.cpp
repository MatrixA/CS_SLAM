#include "Frames.h"

namespace CS_SLAM{

Frames::Frames(){
    KeyFrameDatabase.resize(0);
}



void Frames::add(KeyFrame kf){
    std::unique_lock<std::mutex> lock(mMutex);
    KeyFrameDatabase.push_back(kf);
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

KeyFrame Frames::GetCurrentKeyFrame(){
    std::unique_lock<std::mutex> lock(mMutex);
    return KeyFrameDatabase.back();
}

KeyFrame Frames::GetKeyFrameByID(int id){
    std::unique_lock<std::mutex> lock(mMutex);
    return KeyFrameDatabase[id];
}

std::vector<int> Frames::GetOverlaps(KeyFrame kf, int threshold){
/*
输入：当前扫描对应的位姿x_nk，距离阈值threshold
输出：所有相邻扫描对应的位姿id数组
*/
    Eigen::Vector3d x_nk = kf.GetPos();
    std::vector<int> ans;
    for(int i = 0; i < KeyFrameDatabase.size(); i++){
        double dis = (KeyFrameDatabase[i].GetPos() - x_nk).norm();
        if(dis < threshold){
            ans.push_back(i);
        }
    }
    return ans;
}

std::vector<int> Frames::GetCurrentOverlaps(int threshold){
    Eigen::Vector3d x_nk= GetCurrentKeyFrame().GetPos();
    std::vector<int> ans;
    for(int i = 0; i < KeyFrameDatabase.size(); i++){
        double dis = (KeyFrameDatabase[i].GetPos() - x_nk).norm();
        if(dis < threshold){
            ans.push_back(i);
        }
    }
    return ans;
}


}