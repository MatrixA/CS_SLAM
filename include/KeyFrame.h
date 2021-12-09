#ifndef KEYFRAME_H
#define KEYFRAME_H
/*
CS_SLAM的一部分。

KeyFrame类
关键帧，包括位姿，以及该帧坐标系下的声纳图像和视觉图像。

*/

#include "Eigen/Core"
#include "RandomVector.h"

namespace CS_SLAM{
    
class KeyFrame{
public:
    KeyFrame();
    ~KeyFrame();
    KeyFrame(Eigen::VectorXd kfPos,Eigen::MatrixXd kfPosP);

    Eigen::VectorXd GetPos();
    Eigen::MatrixXd GetPosP();
    void setPos(Eigen::VectorXd);
    void setPosP(Eigen::MatrixXd);
    void Transform(motion transform);

private:
    RandomVector mKfPose;
    // Eigen::MatrixXd* Simg;
    // cv::mat* Cimg;
};

}

#endif // KEYFRAME_H
