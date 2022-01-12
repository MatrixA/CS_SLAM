#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <cstdlib>
#include <pangolin/pangolin.h>
#include "KeyFrame.h"
#include "LocalMap.h"
#include "Frames.h"
#include "Drawer.h"
#include "Utils.h"

namespace CS_SLAM{

class Viewer{

public:
    Viewer();
    Viewer(LocalMap* pMap, Frames* pFrames);
    void Close();
    void UpdateMap();
    void AddCurrentFrame(KeyFrame* current_frame, unsigned long long timestamp_);
    void ThreadLoop();
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);
    // cv::Mat PlotFrameImage();

private:
    // void DrawFrame();
    // void DrawMapPoints();

    Drawer* mpDrawer = nullptr;
    LocalMap* mpMap = nullptr;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;


    std::mutex mMutexViwerData;

    unsigned long long mlTimestamp;
    KeyFrame* mKfCurrent = nullptr;
    KeyFrame* mKfLast = nullptr;
    bool mbInitKf = false;
    bool mbViewerRunning=true;
    bool mbMapUpdated=false;
};


}

#endif //VIEWER_H