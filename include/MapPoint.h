#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "KeyFrame.h"
#include "Eigen/Core"

namespace CS_SLAM
{


class MapPoint{
public:
    MapPoint(){}
    Eigen::Vector2d GetWorldPos();

private:
    Eigen::Vector2d worldPose;
    Eigen::Matrix2d worldPoseP;
    KeyFrame * kfRef;
};


}

#endif