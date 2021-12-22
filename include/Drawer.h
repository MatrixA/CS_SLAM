#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Atlas.h"
#include <pangolin/pangolin.h>

namespace CS_SLAM
{

class Drawer{
public:
    Drawer(Atlas* pAtlas, const std::string &strSettingPath);

    Atlas* mpAtlas;

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