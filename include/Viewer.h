#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>
#include "Atlas.h"

namespace CS_SLAM{

class Viewer{

public:
    Viewer();
    void Close();
    void UpdateMap();
    
private:
    void ThreadLoop();
    void DrawFrame();
    void DrawMapPoints();
    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);
    cv::Mat PlotFrameImage();
    std::thread viewer_thread_;
    
    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    Atlas* mpAtlas;
    std::mutex viewer_data_mutex_;
};


}

#endif //VIEWER_H