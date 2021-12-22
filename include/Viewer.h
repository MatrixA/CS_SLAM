#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>
#include "KeyFrame.h"
#include "Map.h"
#include "Drawer.h"

namespace CS_SLAM{

class Viewer{

public:
    Viewer();
    Viewer(Map* pMap);
    void Close();
    void UpdateMap();
    void AddCurrentFrame(KeyFrame* current_frame);
    
private:
    void ThreadLoop();
    // void DrawFrame();
    // void DrawMapPoints();
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);
    cv::Mat PlotFrameImage();
    std::thread viewer_thread_;

    Drawer* mpDrawer = nullptr;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    Map* mpMap = nullptr;
    std::mutex viewer_data_mutex_;

    KeyFrame* current_frame_ = nullptr;
    bool mbViewerRunning;
    bool mbMapUpdated=false;
};


}

#endif //VIEWER_H