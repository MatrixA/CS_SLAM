#include "Atlas.h"


namespace CS_SLAM
{
    Atlas::Atlas(){}
    Atlas::~Atlas(){}

    void Atlas::AddKeyFrame(KeyFrame* pKF){
        keyFrames.push_back(pKF);
    }
    void Atlas::AddMapPoint(MapPoint* pMP){
        mapPoints.push_back(pMP);
    }

    std::vector<KeyFrame *> Atlas::GetKeyFrames(){
        return keyFrames;
    }

    std::vector<MapPoint *> Atlas::GetMapPoints(){
        return MapPoints;
    }

} // namespace CS_SLAM
