#include "Drawer.h"

namespace CS_SLAM{
    
Drawer::Drawer(LocalMap* pMap):mpMap(pMap){
    // cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // bool is_correct = ParseViewerParamFile(fSettings);

    // if(!is_correct){
    //     std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
    //     try{
    //         throw -1;
    //     }
    //     catch(exception &e){
    //     }
    // }
}


// bool Drawer::ParseViewerParamFile(cv::FileStorage &fSettings){
//     bool b_miss_params = false;
//     cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
//     if(!node.empty()){
//         mKeyFrameSize = node.real();
//     }
//     else{
//         std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["Viewer.KeyFrameLineWidth"];
//     if(!node.empty()){
//         mKeyFrameLineWidth = node.real();
//     }
//     else{
//         std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["Viewer.GraphLineWidth"];
//     if(!node.empty()){
//         mGraphLineWidth = node.real();
//     }
//     else{
//         std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["Viewer.PointSize"];
//     if(!node.empty()){
//         mPointSize = node.real();
//     }
//     else{
//         std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["Viewer.CameraSize"];
//     if(!node.empty()){
//         mCameraSize = node.real();
//     }
//     else{
//         std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }

//     node = fSettings["Viewer.CameraLineWidth"];
//     if(!node.empty()){
//         mCameraLineWidth = node.real();
//     }
//     else{
//         std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
//         b_miss_params = true;
//     }
//     return !b_miss_params;
// }


//Twc存储的是相机的旋转和平移矩阵
void Drawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;
    /**
     * glPushMatrix、glPopMatrix实际上是做入栈和出栈的操作。因为旋转和平移都是在glPushMatrix、glPopMatrix之间进行的。
     * 在绘制之前先把当前的位置保存下来，后边调用glPopMatrix可再次回到当前的位置。下次再绘制的时候就不会受当前的旋转或者平移的影响。
     * glPushMatrix()和glPopMatrix()的配对使用能够消除上一次的变换对本次变换的影响。使本次变换是以世界坐标系的原点为參考点进行。
     * */
    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);//把m指定的16个值作为一个矩阵，与当前矩阵相乘，并把结果存储在当前矩阵中
#endif

    glLineWidth(mCameraLineWidth);
    //相机为绿色
    glColor3f(0.0f,1.0f,0.0f);

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void Drawer::DrawMapPoints(){
    // const std::vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
    // const std::vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();

    // if(vpMPs.empty())
    //     return;

    // pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    // glEnable(GL_DEPTH_TEST);
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // glPointSize(mPointSize);

    // DrawPointCloud(const std::vector<RandomVector> &rvs);
    // glBegin(GL_POINTS);
    // glColor3f(0.0,0.0,0.0);

    // for(size_t i=0, iend=vpMPs.size(); i<iend;i++){
    //     if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
    //         continue;
    //     cv::Mat pos = vpMPs[i]->GetWorldPos();
    //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    // }
    // glEnd();

    // glPointSize(mPointSize);
    // glBegin(GL_POINTS);
    // glColor3f(1.0,0.0,0.0);

    // for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++){
    //     if((*sit)->isBad())
    //         continue;
    //     cv::Mat pos = (*sit)->GetWorldPos();
    //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    // }

    // glEnd();
}

/**
 * @brief Draw RandomVector type points:(xs,ys,0)
 * 
 * @param rvs vectors of RandomVector rype points
 */
void Drawer::DrawSonar() {
    std::vector<point> rvs = (mpFrames->GetCurrentKeyFrame()).GetSonarFullScan();
    if (rvs.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 25, 25, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, 0, 5, 0, 0, 0, pangolin::AxisY)
    );


    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: rvs) {
            glColor3f(0, 0, 0);
            glVertex3d(p.hat[0], p.hat[1], 0);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

/**
 * @brief draw Keyframes and mappoints
 * 
 * @param bDrawKF whether draw Keyframes
 * @param bDrawGraph whether draw mappoints
 * @param bDrawInertialGraph whether draw inertial graph
 */
void Drawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph){
    // const float &w = mKeyFrameSize;
    // const float h = w*0.75;
    // const float z = w*0.6;

    // const std::vector<KeyFrame> vpKFs = mpFrames->GetAllKeyFrames();
    // for(int i=0;i<mpFrames->Size();i++){
    //     KeyFrame tmp = mpFrames->GetKeyFrameByID(i);

    // }
    // if(bDrawKF){
    //     for(int i=0; i<mpFrames->size(); i++)
    //     {
    //         KeyFrame KF = mpFrames->GetKeyFrameByID(i);
    //         Eigen::Vector3d kfPos = KF.GetPos();
            // cv::Mat Twc = KF.GetPoseInverse().t();
    //         unsigned int index_color = pKF->mnOriginMapId;

            // glPushMatrix();

    //         glMultMatrixf(Twc.ptr<GLfloat>(0));

            // if(i == 0) // It is the first KF in the map
            // {
            //     glLineWidth(mKeyFrameLineWidth*5);
            //     glColor3f(1.0f,0.0f,0.0f);
            //     glBegin(GL_LINES);

    //             //cout << "Initial KF: " << mpAtlas->GetCurrentMap()->GetOriginKF()->mnId << endl;
    //             //cout << "Parent KF: " << vpKFs[i]->mnId << endl;
            // }
            // else
            // {
            //     glLineWidth(mKeyFrameLineWidth);
            //     glColor3f(0.0f,0.0f,1.0f);
    //             glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
            //     glBegin(GL_LINES);
            // }

            // glVertex3f(0,0,0);
            // glVertex3f(w,h,z);
            // glVertex3f(0,0,0);
            // glVertex3f(w,-h,z);
            // glVertex3f(0,0,0);
            // glVertex3f(-w,-h,z);
            // glVertex3f(0,0,0);
            // glVertex3f(-w,h,z);

            // glVertex3f(w,h,z);
            // glVertex3f(w,-h,z);

            // glVertex3f(-w,h,z);
            // glVertex3f(-w,-h,z);

            // glVertex3f(-w,h,z);
            // glVertex3f(w,h,z);

            // glVertex3f(-w,-h,z);
            // glVertex3f(w,-h,z);
            // glEnd();

            // glPopMatrix();

            //Draw lines with Loop and Merge candidates
            // glLineWidth(mGraphLineWidth);
            // glColor4f(1.0f,0.6f,0.0f,1.0f);
            // glBegin(GL_LINES);
    //         cv::Mat Ow = pKF->GetCameraCenter();
            // std::vector<int>viLoopCandKFs->GetOverlaps(KF, 0.1);
    //         const vector<KeyFrame*> vpLoopCandKFs = pKF->mvpLoopCandKFs;
            // if(!vpLoopCandKFs.empty())
            // {
                //连接相机和位姿
                // for(auto & p:viLoopCandKFs){
                //     KeyFrame kf = mpFrames->GetKeyFrameByID(p);
                //     // glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                //     // glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                // }
                //连接回环的关键帧
    //             for(vector<KeyFrame*>::const_iterator vit=vpLoopCandKFs.begin(), vend=vpLoopCandKFs.end(); vit!=vend; vit++)
    //             {
    //                 cv::Mat Ow2 = (*vit)->GetCameraCenter();
    //                 glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //                 glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
    //             }
            // }
    //         const vector<KeyFrame*> vpMergeCandKFs = pKF->mvpMergeCandKFs;
    //         if(!vpMergeCandKFs.empty())
    //         {
    //             for(vector<KeyFrame*>::const_iterator vit=vpMergeCandKFs.begin(), vend=vpMergeCandKFs.end(); vit!=vend; vit++)
    //             {
    //                 cv::Mat Ow2 = (*vit)->GetCameraCenter();
    //                 glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //                 glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
    //             }
    //         }*/

    //         glEnd();
    //     }
    // }

    // if(bDrawGraph){
    //     glLineWidth(mGraphLineWidth);
    //     glColor4f(0.0f,1.0f,0.0f,0.6f);
    //     glBegin(GL_LINES);

    //     // cout << "-----------------Draw graph-----------------" << endl;
    //     for(size_t i=0; i<vpKFs.size(); i++)
    //     {
    //         // Covisibility Graph
    //         const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
    //         cv::Mat Ow = vpKFs[i]->GetCameraCenter();
    //         if(!vCovKFs.empty())
    //         {
    //             for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
    //             {
    //                 if((*vit)->mnId<vpKFs[i]->mnId)
    //                     continue;
    //                 cv::Mat Ow2 = (*vit)->GetCameraCenter();
    //                 glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //                 glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
    //             }
    //         }

    //         // Spanning tree
    //         KeyFrame* pParent = vpKFs[i]->GetParent();
    //         if(pParent)
    //         {
    //             cv::Mat Owp = pParent->GetCameraCenter();
    //             glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //             glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
    //         }

    //         // Loops
    //         set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
    //         for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
    //         {
    //             if((*sit)->mnId<vpKFs[i]->mnId)
    //                 continue;
    //             cv::Mat Owl = (*sit)->GetCameraCenter();
    //             glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //             glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
    //         }
    //     }

    //     glEnd();
    // }

    // if(bDrawInertialGraph && mpAtlas->isImuInitialized()){
    //     glLineWidth(mGraphLineWidth);
    //     glColor4f(1.0f,0.0f,0.0f,0.6f);
    //     glBegin(GL_LINES);

    //     //Draw inertial links
    //     for(size_t i=0; i<vpKFs.size(); i++)
    //     {
    //         KeyFrame* pKFi = vpKFs[i];
    //         cv::Mat Ow = pKFi->GetCameraCenter();
    //         KeyFrame* pNext = pKFi->mNextKF;
    //         if(pNext)
    //         {
    //             cv::Mat Owp = pNext->GetCameraCenter();
    //             glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //             glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
    //         }
    //     }

    //     glEnd();
    // }

    // std::vector<LocalMap*> vpMaps = mpAtlas->GetAllMaps();

    // if(bDrawKF){
    //     for(LocalMap* pMap : vpMaps)
    //     {
    //         if(pMap == mpAtlas->GetCurrentMap())
    //             continue;

    //         vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    //         for(size_t i=0; i<vpKFs.size(); i++)
    //         {
    //             KeyFrame* pKF = vpKFs[i];
    //             cv::Mat Twc = pKF->GetPoseInverse().t();
    //             unsigned int index_color = pKF->mnOriginMapId;

    //             glPushMatrix();

    //             glMultMatrixf(Twc.ptr<GLfloat>(0));

    //             if(!vpKFs[i]->GetParent()) // It is the first KF in the map
    //             {
    //                 glLineWidth(mKeyFrameLineWidth*5);
    //                 glColor3f(1.0f,0.0f,0.0f);
    //                 glBegin(GL_LINES);
    //             }
    //             else
    //             {
    //                 glLineWidth(mKeyFrameLineWidth);
    //                 //glColor3f(0.0f,0.0f,1.0f);
    //                 glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
    //                 glBegin(GL_LINES);
    //             }

    //             glVertex3f(0,0,0);
    //             glVertex3f(w,h,z);
    //             glVertex3f(0,0,0);
    //             glVertex3f(w,-h,z);
    //             glVertex3f(0,0,0);
    //             glVertex3f(-w,-h,z);
    //             glVertex3f(0,0,0);
    //             glVertex3f(-w,h,z);

    //             glVertex3f(w,h,z);
    //             glVertex3f(w,-h,z);

    //             glVertex3f(-w,h,z);
    //             glVertex3f(-w,-h,z);

    //             glVertex3f(-w,h,z);
    //             glVertex3f(w,h,z);

    //             glVertex3f(-w,-h,z);
    //             glVertex3f(w,-h,z);
    //             glEnd();

    //             glPopMatrix();
    //         }
    //     }
    // }
}

// void Drawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc){
//     const float &w = mCameraSize;
//     const float h = w*0.75;
//     const float z = w*0.6;

//     glPushMatrix();

// #ifdef HAVE_GLES
//         glMultMatrixf(Twc.m);
// #else
//         glMultMatrixd(Twc.m);
// #endif

//     glLineWidth(mCameraLineWidth);
//     glColor3f(0.0f,1.0f,0.0f);
//     glBegin(GL_LINES);
//     glVertex3f(0,0,0);
//     glVertex3f(w,h,z);
//     glVertex3f(0,0,0);
//     glVertex3f(w,-h,z);
//     glVertex3f(0,0,0);
//     glVertex3f(-w,-h,z);
//     glVertex3f(0,0,0);
//     glVertex3f(-w,h,z);

//     glVertex3f(w,h,z);
//     glVertex3f(w,-h,z);

//     glVertex3f(-w,h,z);
//     glVertex3f(-w,-h,z);

//     glVertex3f(-w,h,z);
//     glVertex3f(w,h,z);

//     glVertex3f(-w,-h,z);
//     glVertex3f(w,-h,z);
//     glEnd();

//     glPopMatrix();
// }

// void Drawer::DrawSonar(){
    // const std::vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
    // const std::vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();

    // if(vpMPs.empty())
    //     return;

    // pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    // glEnable(GL_DEPTH_TEST);
    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // glPointSize(mPointSize);

    // DrawPointCloud(const std::vector<RandomVector> &rvs);
    // glBegin(GL_POINTS);
    // glColor3f(0.0,0.0,0.0);

    // for(size_t i=0, iend=vpMPs.size(); i<iend;i++){
    //     if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
    //         continue;
    //     cv::Mat pos = vpMPs[i]->GetWorldPos();
    //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    // }
    // glEnd();

    // glPointSize(mPointSize);
    // glBegin(GL_POINTS);
    // glColor3f(1.0,0.0,0.0);

    // for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++){
    //     if((*sit)->isBad())
    //         continue;
    //     cv::Mat pos = (*sit)->GetWorldPos();
    //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    // }

    // glEnd();
// }

// void DrawPointCloud(const std::vector<RandomVector> &rvs) {
//     if (rvs.empty()) {
//         std::cerr << "Point cloud is empty!" << std::endl;
//         return;
//     }

//     pangolin::OpenGlRenderState s_cam(
//             pangolin::ProjectionMatrix(1024, 768, 25, 25, 512, 389, 0.1, 1000),
//             pangolin::ModelViewLookAt(0, 0, 5, 0, 0, 0, pangolin::AxisY)
//     );


//     pangolin::View &d_cam = pangolin::CreateDisplay()
//         .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
//         .SetHandler(new pangolin::Handler3D(s_cam));

//     while (pangolin::ShouldQuit() == false) {
//         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//         d_cam.Activate(s_cam);
//         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

//         glPointSize(2);
//         glBegin(GL_POINTS);
//         for (auto &p: rvs) {
//             glColor3f(0, 0, 0);
//             glVertex3d(p.hat[0], p.hat[1], 0);
//         }
//         glEnd();
//         pangolin::FinishFrame();
//         usleep(5000);   // sleep 5 ms
//     }
//     return ;
// }

}