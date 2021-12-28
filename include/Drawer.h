#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "LocalMap.h"
#include <pangolin/pangolin.h>

namespace CS_SLAM
{

class Drawer{
public:
    // Drawer(LocalMap* pMap, const std::string &strSettingPath);
    Drawer(LocalMap* pMap);

    LocalMap* mpMap;

    void DrawMapPoints();
    void DrawSonar();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);

private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;
};

}

#endif