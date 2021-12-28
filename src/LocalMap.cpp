#include "LocalMap.h"


namespace CS_SLAM
{
    LocalMap::LocalMap(){}
    LocalMap::~LocalMap(){}

    void LocalMap::AddKeyFrame(KeyFrame* pKF){
        keyFrames.push_back(pKF);
    }
    void LocalMap::AddMapPoint(MapPoint* pMP){
        mapPoints.push_back(pMP);
    }

    std::vector<KeyFrame *> LocalMap::GetKeyFrames(){
        return keyFrames;
    }

    std::vector<MapPoint *> LocalMap::GetMapPoints(){
        return mapPoints;
    }

} // namespace CS_SLAM
