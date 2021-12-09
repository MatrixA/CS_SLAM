#ifndef MAP_H
#define MAP_H

#include <vector>
#include "MapPoint.h"
#include "KeyFrame.h"

namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class Map{
public:
    Map();
    ~Map();
    void AddMapPoint(MapPoint* pMP);
    void AddKeyFrame(KeyFrame* pKF);
    std::vector<KeyFrame *> Map::GetKeyFrames();
    std::vector<MapPoint *> Map::GetMapPoints();

private:
    std::vector<MapPoint *>mapPoints;
    std::vector<KeyFrame *>keyFrames;
    ScanFormer* mpScanFormer;
};

}

#endif // MAP_H