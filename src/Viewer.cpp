#include "Viewer.h"

namespace CS_SLAM{

Viewer::Viewer(LocalMap* pMap):mpMap(pMap){
    //viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    mpDrawer = new Drawer(pMap);
}

void Viewer::Close(){
    mbViewerRunning = false;
}

void Viewer::AddCurrentFrame(KeyFrame* current_frame){
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap(){
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(mpMap != nullptr);
    // active_keyframes_ = map_->GetActiveKeyFrames();
    // active_landmarks_ = map_->GetActiveMapPoints();
    mbMapUpdated = true;
}

void Viewer::ThreadLoop(){
//     pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
//     glEnable(GL_DEPTH_TEST);
//     glEnable(GL_BLEND);
//     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
// //创建按钮和选择框
//     pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
//     pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
//     pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
//     pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
//     pangolin::Var<bool> mennuShowGraph("menu.Show Graph", true, true);
//     pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
//     pangolin::Var<bool> menuReset("menu.Reset", false, false);


// //设置pangolin相机的投影模型和观测方向
// //ProjectionMatrix(w,h,fu,fv,u0,v0,zNear,zFar)
// //ModelViewLookAt(观测点位置，观测目标位置，观测的方位向量)
//     pangolin::OpenGlRenderState vis_camera(
//         pangolin::ProjectionMatrix(1024,768,400,400,512,384,0.1,1000),
//         pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,0,0,0,0.0,-1.0,0.0));

// //定义显示面板大小
//     pangolin::View& vis_display = 
//         pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
//             .SetHandler(new pangolin::Handler3D(vis_camera));

//     const float blue[3] = {0, 0, 1};
//     const float green[3] = {0, 1, 0};

//     while(!pangolin::ShouldQuit() && mbViewerRunning){
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//         vis_display.Activate(vis_camera);

//         std::unique_lock<std::mutex> lock(viewer_data_mutex_);
//         if(current_frame_){
//             // mpFrameDrawer->DrawFrame(current_frame_, green);
//             mpDrawer->DrawKeyFrames(true,false,false);
//             FollowCurrentFrame(vis_camera);

//             cv::Mat img = PlotFrameImage();
//             cv::imshow("image", img);
//             cv::waitKey(1);
//         }

//         if(mpMap){
//             mpDrawer->DrawMapPoints();
//         }

//         pangolin::FinishFrame();
//         usleep(5000);
//     }
//     std::cout<<"Stop Viewer"<<std::endl;
    // LOG(INFO) << "Stop Viewer";
}





}