#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "LocalMap.h"

namespace CS_SLAM
{

class FrameDrawer{
public:
    FrameDrawer(LocalMap* pMap, const std::string &strSettingPath);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);
private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
    LocalMap* mpMap;
};

}

#endif //FRAMEDRAWER_H