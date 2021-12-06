#ifndef ATLAS_H
#define ATLAS_H

#include <vector>
#include "MapPoint.h"
#include "KeyFrame.h"

namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

public:
    Atlas();
    ~Atlas();
    void AddMapPoint(MapPoint* pMP);
    void AddKeyFrame(KeyFrame* pKF);
    std::vector<KeyFrame *> Atlas::GetKeyFrames();
    std::vector<MapPoint *> Atlas::GetMapPoints();

private:
    std::vector<MapPoint *>mapPoints;
    std::vector<KeyFrame *>keyFrames;
    ScanFormer* mpScanFormer;

}

#endif // ATLAS_H