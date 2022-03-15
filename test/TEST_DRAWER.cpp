#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
//容器vector的头文件，vector是可以存放任意类型的动态数组
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <thread>
//用于显示3D视觉图像
#include <pangolin/pangolin.h>
//Linux系统服务头文件
#include <unistd.h>
#include <cmath>
#include <mutex>

#include "RandomVector.h"
#include "KeyFrame.h"
#include "Frames.h"

#define PI acos(-1)



std::vector<CS_SLAM::point>sonarPoints;
std::vector<CS_SLAM::point>distortedPoints;
std::vector<CS_SLAM::pose>poses;
cv::Mat cameraImg;

// void Import(const std::string& filename, std::vector<CS_SLAM::MeasurementPackage> &data, std::vector<int> &data_ind, enum CS_SLAM::MeasurementPackage::SensorType sensor, bool tags){
//     std::ifstream infile;
//     infile.open(filename, std::ios::in);
//     std::cout<<"Reading"<<std::endl;
//     if(!infile.is_open()){
//         std::cout<<"open file fail"<<std::endl;
//         return;
//     }

//     return ;
// }

void initSonarPoints(){
    sonarPoints.resize(100);
    int i = 0;
    for(auto&p: sonarPoints){
        p.hat(0)=50.0*rand()/RAND_MAX;
        p.hat(1)=0.03*(i++);
        p.hat(2)=rand()*128;
    }
}

int main(){
    initSonarPoints();

    
    CS_SLAM::Frames* fdbs = new CS_SLAM::Frames();
    //读入文件，加入Frames
    fdbs->Init2DFromFile("/home/fernando/Code/CS_SLAM/poses.txt", false);
    std::cout<<fdbs->Size()<<" poses."<<std::endl;
    pangolin::CreateWindowAndBind("Main",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(320,240,200,200,160,120,0.1,1000);
    pangolin::OpenGlRenderState s_cam_pose(
            pangolin::ProjectionMatrix(768, 512, 384, 256, 120, 0, 0.1,1000),
            pangolin::ModelViewLookAt(0, 0, 50, 0, 0, 0, pangolin::AxisY)
    );

    pangolin::OpenGlRenderState s_cam_sonar(
            pangolin::ProjectionMatrix(640,480,160,160,320,480,0.1,1000),
            pangolin::ModelViewLookAt(0, 0, 25, 0, 0, 0, pangolin::AxisY)
    );

    pangolin::View& d_cam1 = pangolin::Display("cam1")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetHandler(new pangolin::Handler3D(s_cam_pose));

    pangolin::View& d_cam2 = pangolin::Display("cam2")
        .SetBounds(0.5, 1.0, 0.0, 1.0)
        .SetAspect(640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(s_cam_sonar));

    pangolin::View& d_img2 = pangolin::Display("img2")
        .SetBounds(0.1, 1.0, 0.5, 1.0)
        .SetAspect(640.0f/480.0f);

    pangolin::View& d_img3 = pangolin::Display("img3")
        .SetBounds(0.0, pangolin::Attach::Pix(392), 0.0, pangolin::Attach::Pix(1082));

    // pangolin::View& d_img4 = pangolin::Display("img4")
    //     .SetAspect(640.0f/480.0f);

    pangolin::Display("pose")
        .SetBounds(0.0, 1.0, 0.0, 0.5)
        // .AddDisplay(d_cam1)
        .AddDisplay(d_cam1);
        // .AddDisplay(d_img2)
        // .AddDisplay(d_img3);

    pangolin::Display("sonar")
        .SetBounds(0.0, 1.0, 0.5, 1.0)
        // .AddDisplay(d_cam1)
        .AddDisplay(d_cam2)
        // .AddDisplay(d_img2)
        .AddDisplay(d_img3);
        // .AddDisplay(d_img4);
    
    const int width =  64;
    const int height = 48;


    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        d_cam1.Activate(s_cam_pose);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.29f, 0.71f, 1.0f);
        //画位姿
        // pangolin::glDrawColouredCube();
        glBegin(GL_LINE_LOOP);
        int cnt=0;
        while(cnt < fdbs->Size()){
            Eigen::Vector3d tmpPoseHat = fdbs->GetKeyFrameByID(cnt,false)->GetPose().hat;
            glColor3f(0.4f,0.4f,0.2f);
            glPointSize(4);
            glVertex3d(tmpPoseHat(0), tmpPoseHat(1), 0);
            cnt++;
        }
        
        glEnd();
        d_cam2.Activate(s_cam_sonar);
        //画声纳源数据
        glBegin(GL_LINE_STRIP);
        for(CS_SLAM::point &p:sonarPoints){
            double r=p.hat(0),theta=p.hat(1);
            glColor3f(0.4f,0.4f,0.2f);
            glPointSize(4);
            glVertex2d(r*cos(theta),r*sin(theta));
            // std::cout<<r<<","<<theta<<","<<r*cos(theta)<<","<<r*sin(theta)<<std::endl;
        }
        glEnd();

        d_img2.Activate();
        //画声纳处理后的数据
        //
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        // //画图像源数据
        pangolin::GlTexture cameraImgTexture(1082/* d */, 392, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
        cameraImg = cv::imread("/home/fernando/Code/CODE/1.png");
        cameraImgTexture.Upload(cameraImg.data, GL_BGR, GL_UNSIGNED_BYTE);
        d_img3.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);
        cameraImgTexture.RenderToViewportFlipY();

        // d_img4.Activate();
        // 画图像处理后的数据
        //

        pangolin::FinishFrame();
    }

    return 0;
}
