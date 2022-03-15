#ifndef SYSTEM_H
#define SYSTEM_H

#include <pangolin/pangolin.h>
#include <string>
#include <fstream>
#include <thread>

#include "yaml-cpp/yaml.h"
#include "MSCKF.h"
#include "LocalMap.h"
#include "Atlas.h"
#include "Viewer.h"
#include "ScanFormer.h"
#include "MeasurementPackage.h"
#include "RandomVector.h"
#include "LocalMapper.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frames.h"

namespace CS_SLAM
{
static const std::string window_name = "MyPangolin";
class MeasurementPackage;
class MSCKF;

class System
{
public:
    System();
    System(YAML::Node sensorConfig);
    ~System();
    void Initialize();
    void TrackAHRS(MeasurementPackage meas);
    void TrackDVL(MeasurementPackage meas);
    void TrackSonar(MeasurementPackage meas);
    void TrackDS(MeasurementPackage meas);
    void TrackMono(MeasurementPackage meas);
    void SetUp();
    //void Plot();
    // void PlotTrajectory();
    void Reset();
    void SaveTrajectory(const std::string &filename);
    void SaveTrajectoryFromDatabase(const std::string &filename);
    int GetTrackingState();
    bool isLost();
    bool isFinished();
    bool isInitialized();
private:
    unsigned long long timestamp_now=0;
    EKF* mpEKF;
    MSCKF* mpMSCKF; //MSCKF里保存了所有KeyFrames的坐标及其方差
    ScanFormer* mpScanFormer;
    int scnt=0;
    bool mbSetUp = false;
    bool mbReset = false;
    bool mbInit = false;

    YAML::Node mSensorConfig;

    std::thread* mptViewer;
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    LocalMapper* mpLocalMapper;
    Viewer* mpViewer;
    Atlas* mpAtlas;
    LocalMap* mpMap;
    LoopClosing* mpLoopClosing;
    Frames* mpFramesDatabase;
    
    //std::thread viewer_thread_;
};


}

#endif //SYSTEM_H