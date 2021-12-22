#ifndef ATLAS_H
#define ATLAS_H

#include <vector>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ScanFormer.h"

namespace CS_SLAM
{
class MapPoint;
class KeyFrame;

class Atlas{

public:
    Atlas();
    ~Atlas();
    void AddMapPoint(MapPoint* pMP);
    void AddKeyFrame(KeyFrame* pKF);
    std::vector<KeyFrame *> GetKeyFrames();
    std::vector<MapPoint *> GetMapPoints();

private:
    std::vector<MapPoint *>mapPoints;
    std::vector<KeyFrame *>keyFrames;
    ScanFormer* mpScanFormer;

};

}

#endif // ATLAS_H