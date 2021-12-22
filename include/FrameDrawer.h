#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Map.h"

namespace CS_SLAM
{

class FrameDrawer{
public:
    FrameDrawer(Map* pMap, const std::string &strSettingPath);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    Map* mpMap;
};

}

#endif //FRAMEDRAWER_H