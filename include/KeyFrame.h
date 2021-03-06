#ifndef KEYFRAME_H
#define KEYFRAME_H
/*
CS_SLAM的一部分。

KeyFrame类
关键帧，包括位姿，以及该帧坐标系下的声纳图像和视觉图像。

*/
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "RandomVector.h"
#include "MapPoint.h"

namespace CS_SLAM{
    
class KeyFrame{
public:
    KeyFrame();
    ~KeyFrame();
    KeyFrame(pose kf);
    KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP);

    void SetTimeStamp(unsigned long long timestamp);
    unsigned long long GetTimeStamp();
    pose GetPose();
    void SetPose(pose kfPose);
    const std::vector<point>& GetSonarFullScan();
    void SetSonarFullScan(std::vector<point> fs);
    // const std::vector<Eigen::VectorXd>& GetSonarMeasurements();
    void SetSonarMeasurements(const std::vector<Eigen::VectorXd>& fsm);
    const bool HaveSonarFullScan();
    void LoadCameraImg(std::string filename);
    const cv::Mat& GetCameraImage();
    const bool HaveCameraImage();

    void Transform(motion transform);
    void Print();

private:
    unsigned long long mTimeStamp;
    pose mKfPose;
    std::vector<point> mvSonarFullScan;
    std::vector<Eigen::VectorXd> mvFsm;
    // Eigen::MatrixXd* Simg;
    cv::Mat mCimg;
};

}

#endif // KEYFRAME_H
