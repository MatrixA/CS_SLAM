#ifndef SYSTEM_H
#define SYSTEM_H

#include <pangolin/pangolin.h>
#include <string>
#include <thread>

#include "ASEKF.h"
#include "ScanFormer.h"
#include "MeasurementPackage.h"
#include "RandomVector.h"

namespace CS_SLAM
{
static const std::string window_name = "MyPangolin";
class MeasurementPackage;
class ASEKF;

class System
{
public:
    System();
    ~System();
    void TrackAHRS(MeasurementPackage meas,Eigen::VectorXd paramAHRS);
    void TrackDVL(MeasurementPackage meas,Eigen::VectorXd paramDVL);
    void TrackSonar(MeasurementPackage meas,Eigen::VectorXd paramSonar);
    void TrackDS(MeasurementPackage meas,Eigen::VectorXd paramDS);
    void SetUp();
    //void Plot();
    void PlotTrajectory();

private:
    unsigned long long timestamp_now;
    ASEKF asekf; //ASEKF里保存了所有KeyFrames的坐标及其方差
    ScanFormer scanFormer;
    int scnt=0;
    bool isSetUp=false;
    std::thread* mptViewer;
    //std::thread viewer_thread_;
};


}

#endif