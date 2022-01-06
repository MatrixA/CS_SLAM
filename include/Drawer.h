#ifndef DRAWER_H
#define DRAWER_H

#include "LocalMap.h"
#include "Frames.h"
#include <pangolin/pangolin.h>

namespace CS_SLAM
{

class Drawer{
public:
    // Drawer(LocalMap* pMap, const std::string &strSettingPath);
    Drawer(LocalMap* pMap);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawMapPoints();
    void DrawSonar();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);

private:
    float mCameraSize;
    float mCameraLineWidth;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mPointSize;

    LocalMap* mpMap;  //DrawMapPoints的数据源
    Frames* mpFrames;  //DrawKeyFrames的数据源

};

}

#endif