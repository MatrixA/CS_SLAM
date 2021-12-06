#include "MapPoint.h"
#include "KeyFrame.h"
#include "Eigen/Core"

namespace CS_SLAM
{

Eigen::Vector2d MapPoint::GetWorldPos(){
    return worldPose;
}



}