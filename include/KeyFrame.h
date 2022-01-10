#ifndef KEYFRAME_H
#define KEYFRAME_H
/*
CS_SLAM的一部分。

KeyFrame类
关键帧，包括位姿，以及该帧坐标系下的声纳图像和视觉图像。

*/

#include "Eigen/Core"
#include "RandomVector.h"
#include "MapPoint.h"

namespace CS_SLAM{
    
class KeyFrame{
public:
    KeyFrame();
    ~KeyFrame();
    KeyFrame(pose kf);
    KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP);

    pose GetPose();
    void SetPose(pose kfPose);
    const std::vector<point>& GetSonarFullScan();
    void SetSonarFullScan(std::vector<point> fs);
    const std::vector<Eigen::VectorXd>& GetSonarMeasurements();
    void SetSonarMeasurements(const std::vector<Eigen::VectorXd>& fsm);

    void Transform(motion transform);
    void Print();

private:
    pose mKfPose;
    std::vector<point> mvSonarFullScan;
    std::vector<Eigen::VectorXd> mvFsm;
    // Eigen::MatrixXd* Simg;
    // cv::mat* Cimg;
};

}

#endif // KEYFRAME_H
