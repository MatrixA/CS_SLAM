#include "Map.h"


namespace CS_SLAM
{
    Map::Map(){}
    Map::~Map(){}

    void Map::AddKeyFrame(KeyFrame* pKF){
        keyFrames.push_back(pKF);
    }
    void Map::AddMapPoint(MapPoint* pMP){
        mapPoints.push_back(pMP);
    }

    std::vector<KeyFrame *> Map::GetKeyFrames(){
        return keyFrames;
    }

    std::vector<MapPoint *> Map::GetMapPoints(){
        return mapPoints;
    }

} // namespace CS_SLAM
