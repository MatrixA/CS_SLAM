#include "Viewer.h"

namespace CS_SLAM{

/**
 * @brief Construct a new Viewer:: Viewer object
 * 
 * @param pMap - localmap for display
 * @param pFrames - frames database for display
 */
Viewer::Viewer(LocalMap* pMap, Frames* pFrames):mpMap(pMap){
    // viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    mpDrawer = new Drawer(pMap, pFrames);
    mViewpointX = 0; mViewpointY = 0; mViewpointZ = 100;
    mViewpointF = 10;
}

/**
 * @brief do sth when Viewer close
 * 
 */
void Viewer::Close(){
    mbViewerRunning = false;
}

/**
 * @brief refresh current keyframe and timestamp to Viewer
 * 
 * @param timestamp_ - transport timestamp to Viewer
 */
void Viewer::RefreshCurrentFrame(unsigned long long timestamp_){
    std::unique_lock<std::mutex> lock(mMutexViwerData);
    mKfCurrent = mpDrawer->GetCurrentFrame();
    if(mKfCurrent==nullptr)std::cout<<"empth current keyframe"<<std::endl;
    mlTimestamp = timestamp_;
}

void Viewer::UpdateMap(){
    std::unique_lock<std::mutex> lock(mMutexViwerData);
    assert(mpMap != nullptr);
    // active_keyframes_ = map_->GetActiveKeyFrames();
    // active_landmarks_ = map_->GetActiveMapPoints();
    mbMapUpdated = true;
}

/**
 * @brief keep current framed centered in the screen
 * 
 * @param vis_camera 
 */
void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera){
    // mKfCurrent->Print();
    Eigen::Matrix4d dos = (mKfCurrent->GetPose()).toSE3().matrix();
    pangolin::OpenGlMatrix m(dos);
    vis_camera.Follow(m, true);
}

/**
 * @brief Viewer's main loop
 * 
 */
void Viewer::ThreadLoop(){
    pangolin::CreateWindowAndBind("MySLAM", 1710, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//创建按钮和选择框
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));

    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> mennuShowGraph("menu.Show Graph", true, true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<std::string> menuTimeStamp("menu.timestamp", std::to_string(mlTimestamp));
//设置pangolin相机的投影模型和观测方向
//ProjectionMatrix(w,h,fu,fv,u0,v0,zNear,zFar)
//ModelViewLookAt(观测点位置，观测目标位置，观测的方位向量)
    pangolin::OpenGlRenderState vis_pose(
        pangolin::ProjectionMatrix(1024,768,400,400,512,384,0.1,1000),
        pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ,0,0,0,pangolin::AxisY));

    pangolin::View& d_display = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 0.7, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(vis_pose));

    pangolin::OpenGlRenderState vis_sonar(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,1000),
            pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, pangolin::AxisY)
    );

    pangolin::View& d_sonar = pangolin::CreateDisplay()
        .SetBounds(0.5, 1.0, 0.0, 1.0)
        .SetAspect(1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(vis_sonar));

    pangolin::View& d_camera = pangolin::CreateDisplay()
        .SetBounds(0, 0.5, 0.0, 1.0)
        // .SetAspect(1024.0f/768.0f)
        .SetLock(pangolin::LockRight, pangolin::LockBottom);

    pangolin::Display("Camera_Sonar")
        .SetBounds(0,1,0.7,1)
        .AddDisplay(d_sonar)
        .AddDisplay(d_camera);

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};
    const float red[3] = {1, 0, 0};

    while(!pangolin::ShouldQuit() && mbViewerRunning){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        menuTimeStamp=Utils::TimeStamp2TimeString(mlTimestamp);
        
        std::unique_lock<std::mutex> lock(mMutexViwerData);
        d_display.Activate(vis_pose);
        if(mKfCurrent){
            mpDrawer->DrawFrame(mKfCurrent, blue, menuShowKeyFrames, menuShowPoints);//画关键帧以及观测点
            if(menuFollowCamera){
                FollowCurrentFrame(vis_pose);//视角移动
            }
            // mpDrawer->DrawKeyFrames(true,false,false);

            //画声纳观测数据
            // d_sonar.Activate(vis_sonar);
            // // std::cout<<"have sonar? "<<(mKfCurrent->GetSonarFullScan()).size()<<std::endl;
            // if(mKfCurrent->HaveSonarFullScan()){
            //     // std::cout<<"sonar full scan"<<(mKfCurrent->GetSonarFullScan()).size()<<std::endl;
            //     mpDrawer->DrawSonar(mKfCurrent);//画声纳图像
            //     mKfLast = mKfCurrent;
            //     mbInitKf = true;
            // }else if(mbInitKf){
            //     mpDrawer->DrawSonar(mKfLast);//画声纳图像
            // }
            // std::cout<<"have "<<(mKfCurrent->GetCameraImage().data != nullptr)<<std::endl;
            if(mKfCurrent->HaveCameraImage()){
                // std::cout<<"have pic"<<std::endl;
                d_camera.Activate();
                glColor3f(1.0f, 1.0f, 1.0f);
                mpDrawer->PlotImage(mKfCurrent);//画相机
                // mKfLast = mKfCurrent;
                // mbInitKf = true;
            }
            // else if(mbInitKf && mKfLast->HaveSonarFullScan()){
            //     std::cout<<"draw backed Sonar"<<std::endl;
            //     mpDrawer->DrawSonar(mKfLast);//draw backed sonar image
            // }

            // std::cout<<"ok?"<<std::endl;
            // cv::Mat img = PlotFrameImage();
            // cv::imshow("image", img);
            // cv::waitKey(1);
        }

        if(mpMap){
            d_display.Activate(vis_pose);
            mpDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowPoints,false);
            // mpDrawer->DrawFrame();
            // mpDrawer->DrawMapPoints();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
    std::cout<<"Stop Viewer"<<std::endl;
    // LOG(INFO) << "Stop Viewer";
}





}