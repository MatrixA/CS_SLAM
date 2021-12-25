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
    void ThreadLoop();
    // void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);
    // cv::Mat PlotFrameImage();


private:
    
    // void DrawFrame();
    // void DrawMapPoints();


    Drawer* mpDrawer = nullptr;
    Map* mpMap = nullptr;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    
    std::mutex viewer_data_mutex_;

    KeyFrame* current_frame_ = nullptr;
    bool mbViewerRunning;
    bool mbMapUpdated=false;
};


}

#endif //VIEWER_H