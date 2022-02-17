#ifndef DRAWER_H
#define DRAWER_H

#include "LocalMap.h"
#include "Frames.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>

namespace CS_SLAM
{

class Drawer{
public:
    // Drawer(LocalMap* pMap, const std::string &strSettingPath);
    Drawer(LocalMap* pMap, Frames* pFrames);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawMapPoints(KeyFrame* kf, const float * color);
    void DrawSonar(KeyFrame *kf);
    void DrawFrame(KeyFrame *kf, const float* color, const bool bDrawKeyFrames, const bool bDrawSonarPoints);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawSonarPoints, const bool bDrawInertialGraph);
    void PlotImage(KeyFrame *kf);
    KeyFrame* GetCurrentFrame();

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